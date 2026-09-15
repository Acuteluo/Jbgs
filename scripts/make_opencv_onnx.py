#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Ultralytics YOLOv8 ONNX 改成 OpenCV 4.5.4 DNN 能加载、能前向的图。

本机 ROS Humble 的 OpenCV 是 4.5.4, 原版 crack_*_best.onnx 会在
`/model.22/Add`(torch.chunk 导出的 Shape/Gather/Add) 上触发 parseBias
`blob_0.size == blob_1.size` 断言, 左右相机因此降级透传。

本脚本只做等价图变换, 不改卷积权重、不改输出约定
([1, 5|37, 8400] + 分割原型 [1, 32, 160, 160]):

  1. 常量折叠静态 Shape / Gather / Add / Div / Mul / Slice 起止,
     去掉 OpenCV 4.5.4 无法广播折叠的动态 chunk 子图;
  2. 把 `boxes[1,4,8400] * strides[1,8400]` 改成先把 8400 维换到
     channel 轴再 Mul。OpenCV 4.5.4 的 Mul(常量) 会变成 Scale,
     Scale 只认 axis=1, 否则前向在 shape_utils::total 断言失败。

用法(系统 python3, 需 onnx; 不参与 colcon 构建):
  /usr/bin/python3 scripts/make_opencv_onnx.py
  /usr/bin/python3 scripts/make_opencv_onnx.py models/crack_seg_best.onnx
"""
from __future__ import annotations

import os
import sys
import tempfile

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper, shape_inference

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _to_np(t) -> np.ndarray:
    return numpy_helper.to_array(t)


def _from_np(name: str, arr: np.ndarray):
    return numpy_helper.from_array(np.asarray(arr), name=name)


def _const_value(node) -> np.ndarray | None:
    for a in node.attribute:
        if a.name == "value":
            return _to_np(a.t)
        if a.name == "value_int":
            return np.array(a.i, dtype=np.int64)
        if a.name == "value_ints":
            return np.array(list(a.ints), dtype=np.int64)
        if a.name == "value_float":
            return np.array(a.f, dtype=np.float32)
    return None


def _static_shape(vi) -> tuple | None:
    dims = []
    for d in vi.type.tensor_type.shape.dim:
        if d.dim_value > 0:
            dims.append(int(d.dim_value))
        else:
            return None
    return tuple(dims)


def _attr(node, name, default=None):
    for a in node.attribute:
        if a.name == name:
            if a.type == onnx.AttributeProto.INT:
                return int(a.i)
            if a.type == onnx.AttributeProto.INTS:
                return list(a.ints)
            if a.type == onnx.AttributeProto.FLOAT:
                return float(a.f)
    return default


def _onnx_div(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if np.issubdtype(a.dtype, np.integer) and np.issubdtype(b.dtype, np.integer):
        out = np.trunc(a.astype(np.float64) / b.astype(np.float64))
        return out.astype(a.dtype)
    return (a / b).astype(np.result_type(a, b, np.float32))


def _collect_shapes(graph) -> dict:
    shapes = {}
    for vi in list(graph.input) + list(graph.output) + list(graph.value_info):
        sh = _static_shape(vi)
        if sh is not None:
            shapes[vi.name] = sh
    return shapes


def fold_static_subgraph(model: onnx.ModelProto) -> onnx.ModelProto:
    """折叠 Shape/chunk 等纯常量子图, 消除 parseBias 的尺寸断言。"""
    model = shape_inference.infer_shapes(model)
    g = model.graph

    consts: dict[str, np.ndarray] = {}
    for init in g.initializer:
        consts[init.name] = _to_np(init)
    shapes = _collect_shapes(g)

    remaining = []
    for node in g.node:
        if node.op_type == "Constant":
            val = _const_value(node)
            if val is None:
                remaining.append(node)
                continue
            consts[node.output[0]] = val
            shapes[node.output[0]] = tuple(val.shape)
        else:
            remaining.append(node)

    changed = True
    safety = 0
    while changed and safety < 64:
        safety += 1
        changed = False
        kept = []
        for node in remaining:
            folded = None
            try:
                if node.op_type == "Shape" and node.input[0] in shapes:
                    folded = np.asarray(shapes[node.input[0]], dtype=np.int64)
                elif node.input and all(inp in consts for inp in node.input):
                    ins = [consts[i] for i in node.input]
                    if node.op_type == "Gather":
                        folded = np.take(ins[0], ins[1], axis=_attr(node, "axis", 0))
                    elif node.op_type == "Add":
                        folded = np.add(ins[0], ins[1])
                    elif node.op_type == "Sub":
                        folded = np.subtract(ins[0], ins[1])
                    elif node.op_type == "Mul":
                        folded = np.multiply(ins[0], ins[1])
                    elif node.op_type == "Div":
                        folded = _onnx_div(ins[0], ins[1])
                    elif node.op_type == "Unsqueeze":
                        axes = _attr(node, "axes")
                        if axes is None and len(ins) > 1:
                            axes = ins[1].tolist()
                        folded = ins[0]
                        for ax in sorted(int(a) for a in axes):
                            folded = np.expand_dims(folded, ax)
                    elif node.op_type == "Squeeze":
                        axes = _attr(node, "axes")
                        if axes is None and len(ins) > 1:
                            axes = tuple(int(a) for a in ins[1].tolist())
                        elif axes:
                            axes = tuple(int(a) for a in axes)
                        else:
                            axes = None
                        folded = np.squeeze(ins[0], axis=axes)
                    elif node.op_type == "Reshape":
                        folded = np.reshape(ins[0], [int(x) for x in ins[1]])
                    elif node.op_type == "Concat":
                        folded = np.concatenate(ins, axis=_attr(node, "axis", 0))
                    elif node.op_type == "Cast":
                        np_dtype = {
                            TensorProto.FLOAT: np.float32,
                            TensorProto.FLOAT16: np.float16,
                            TensorProto.INT64: np.int64,
                            TensorProto.INT32: np.int32,
                            TensorProto.BOOL: np.bool_,
                        }.get(_attr(node, "to"))
                        folded = ins[0].astype(np_dtype) if np_dtype else None
                    elif node.op_type == "Slice":
                        data = ins[0]
                        starts = ins[1].astype(np.int64).tolist()
                        ends = ins[2].astype(np.int64).tolist()
                        axes = (
                            ins[3].astype(np.int64).tolist()
                            if len(ins) > 3
                            else list(range(len(starts)))
                        )
                        steps = (
                            ins[4].astype(np.int64).tolist()
                            if len(ins) > 4
                            else [1] * len(starts)
                        )
                        sl = [slice(None)] * data.ndim
                        for ax, st, en, sp in zip(axes, starts, ends, steps):
                            sl[int(ax)] = slice(int(st), int(en), int(sp))
                        folded = data[tuple(sl)]
            except Exception as exc:
                print(f"  skip fold {node.op_type} {node.name}: {exc}")
                folded = None

            if folded is not None:
                arr = np.asarray(folded)
                consts[node.output[0]] = arr
                shapes[node.output[0]] = tuple(arr.shape)
                changed = True
            else:
                kept.append(node)
        remaining = kept

    used = set()
    for node in remaining:
        used.update(node.input)
    for item in list(g.output) + list(g.input):
        used.add(item.name)

    new_inits = []
    seen = set()
    for name, arr in consts.items():
        if name not in used or name in seen:
            continue
        seen.add(name)
        new_inits.append(_from_np(name, arr))
    for init in g.initializer:
        if init.name not in seen and init.name in used:
            new_inits.append(init)
            seen.add(init.name)

    del g.node[:]
    g.node.extend(remaining)
    del g.initializer[:]
    g.initializer.extend(new_inits)
    return model


def rewrite_scale_broadcast_muls(model: onnx.ModelProto) -> onnx.ModelProto:
    """把 Mul(变量, 常量) 里"常量长度 == 非 channel 维"的广播改到 axis=1。"""
    model = shape_inference.infer_shapes(model)
    g = model.graph
    inits = {i.name: _to_np(i) for i in g.initializer}
    shapes = _collect_shapes(g)

    new_nodes = []
    n_rewrite = 0
    for node in list(g.node):
        if node.op_type != "Mul" or len(node.input) != 2:
            new_nodes.append(node)
            continue
        a, b = node.input[0], node.input[1]
        const_name = var_name = None
        if a in inits and b not in inits:
            const_name, var_name = a, b
        elif b in inits and a not in inits:
            const_name, var_name = b, a
        else:
            new_nodes.append(node)
            continue

        carr = inits[const_name]
        vshape = shapes.get(var_name)
        if vshape is None or carr.size <= 1 or len(vshape) < 2:
            new_nodes.append(node)
            continue
        # OpenCV Scale 默认沿 C(axis=1); 已对齐则不必改
        if vshape[1] == carr.size:
            new_nodes.append(node)
            continue
        axis = None
        for i, dim in enumerate(vshape):
            if dim == carr.size:
                axis = i
                break
        if axis is None or axis == 1:
            new_nodes.append(node)
            continue

        print(
            f"  rewrite {node.name}: {var_name}{vshape} * "
            f"{const_name}{carr.shape} -> transpose axis {axis} to C"
        )
        nd = len(vshape)
        perm = list(range(nd))
        perm[1], perm[axis] = perm[axis], perm[1]
        inv = [0] * nd
        for i, p in enumerate(perm):
            inv[p] = i
        t_in = node.name + "__ocv_tin"
        t_mul = node.name + "__ocv_mul"
        new_nodes.append(helper.make_node(
            "Transpose", [var_name], [t_in],
            name=node.name + "__ocv_T1", perm=perm))
        mul_inputs = [t_in, const_name] if var_name == node.input[0] else [
            const_name, t_in]
        new_nodes.append(helper.make_node(
            "Mul", mul_inputs, [t_mul], name=node.name + "__ocv_M"))
        new_nodes.append(helper.make_node(
            "Transpose", [t_mul], list(node.output),
            name=node.name + "__ocv_T2", perm=inv))
        n_rewrite += 1

    del g.node[:]
    g.node.extend(new_nodes)
    if n_rewrite:
        print(f"  stride-Mul rewrites: {n_rewrite}")
    return model


def convert(src: str, dst: str) -> None:
    print(f"load {src}")
    model = onnx.load(src)
    n0 = len(model.graph.node)
    model = fold_static_subgraph(model)
    model = rewrite_scale_broadcast_muls(model)
    n1 = len(model.graph.node)
    print(f"  nodes {n0} -> {n1}")
    onnx.checker.check_model(model)
    onnx.save(model, dst)
    print(f"  saved {dst}")


def verify_opencv(path: str) -> None:
    import cv2

    net = cv2.dnn.readNetFromONNX(path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    blob = np.zeros((1, 3, 640, 640), np.float32)
    net.setInput(blob)
    names = net.getUnconnectedOutLayersNames()
    outs = net.forward(names)
    for i, out in enumerate(outs):
        print(f"  verify {names[i]} shape={out.shape} finite={np.isfinite(out).all()}")


def main() -> int:
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        files = [
            os.path.join(REPO_ROOT, "models", "crack_seg_best.onnx"),
            os.path.join(REPO_ROOT, "models", "crack_best.onnx"),
        ]

    for src in files:
        if not os.path.isfile(src):
            print(f"找不到模型: {src}", file=sys.stderr)
            return 1
        fd, tmp = tempfile.mkstemp(suffix=".onnx", dir=os.path.dirname(src))
        os.close(fd)
        try:
            convert(src, tmp)
            verify_opencv(tmp)
            os.replace(tmp, src)
            print(f"updated {src}")
        except Exception:
            if os.path.exists(tmp):
                os.remove(tmp)
            raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
