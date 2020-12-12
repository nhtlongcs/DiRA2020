#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#!/usr/bin/env python3
import onnx_graphsurgeon as gs
import onnx
import numpy as np
import yaml
from pathlib import Path


def create_and_add_plugin_node(graph, cfg):
    batch_size = graph.inputs[0].shape[0]
    tensors = graph.tensors()
    boxes_tensor = tensors["boxes"]
    confs_tensor = tensors["confs"]

    keepTopK = int(cfg["keepTopK"])

    num_detections = gs.Variable(name="num_detections").to_variable(
        dtype=np.int32, shape=[batch_size, 1]
    )
    nmsed_boxes = gs.Variable(name="nmsed_boxes").to_variable(
        dtype=np.float32, shape=[batch_size, keepTopK, 4]
    )
    nmsed_scores = gs.Variable(name="nmsed_scores").to_variable(
        dtype=np.float32, shape=[batch_size, keepTopK]
    )
    nmsed_classes = gs.Variable(name="nmsed_classes").to_variable(
        dtype=np.float32, shape=[batch_size, keepTopK]
    )

    new_outputs = [num_detections, nmsed_boxes, nmsed_scores, nmsed_classes]

    mns_node = gs.Node(
        op="BatchedNMSDynamic_TRT",
        attrs={
            "shareLocation": 1,
            "backgroundLabelId": -1,
            "numClasses": int(cfg["nbCls"]),
            "topK": int(cfg["topK"]),
            "keepTopK": keepTopK,
            "scoreThreshold": float(cfg["scoreThreshold"]),
            "iouThreshold": float(cfg["iouThreshold"]),
            "isNormalized": 1,
            "clipBoxes": 1,
            "plugin_version": "1",
        },
        inputs=[boxes_tensor, confs_tensor],
        outputs=new_outputs,
    )

    graph.nodes.append(mns_node)
    graph.outputs = new_outputs

    return graph.cleanup().toposort()


if __name__ == "__main__":
    cfgPath = Path("data/parameters.yaml")
    with cfgPath.open() as ifs:
        try:
            cfg = yaml.safe_load(ifs)
        except yaml.YAMLError as err:
            raise ValueError("Could not load params", err)

    onnxPath = Path(cfg["onnxFile"]).with_suffix(".onnx")
    if not onnxPath.exists():
        raise FileNotFoundError("Model not exists")
    if "nms" in str(onnxPath):
        exit(0)

    graph = gs.import_onnx(onnx.load(onnxPath))

    cfg["batchSize"] = graph.inputs[0].shape[0]
    cfg["channel"] = graph.inputs[0].shape[1]
    assert cfg["channel"] == 3
    cfg["height"] = graph.inputs[0].shape[2]
    cfg["width"] = graph.inputs[0].shape[3]

    graph = create_and_add_plugin_node(graph, cfg)
    print(f"Insert NMS layer to model {onnxPath} successfully")

    outPath = onnxPath.with_name(str(onnxPath.stem) + "_nms.onnx")
    with cfgPath.open("w") as ofs:
        yaml.safe_dump(cfg, ofs, sort_keys=False)
    onnx.save(gs.export_onnx(graph), outPath)

    print(f"Saved new model")
