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
import argparse
import onnx_graphsurgeon as gs
import onnx
import numpy as np
from pathlib import Path
import re


def create_and_add_plugin_node(graph, args):
    batch_size = graph.inputs[0].shape[0]
    tensors = graph.tensors()
    boxes_tensor = tensors["boxes"]
    confs_tensor = tensors["confs"]
    keepTopK = int(args.keepTopK)

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
            "numClasses": int(args.nbCls),
            "topK": int(args.topK),
            "keepTopK": keepTopK,
            "scoreThreshold": float(args.score),
            "iouThreshold": float(args.iou),
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


parser = argparse.ArgumentParser(
    description="Script to add nms layer to Yolo ONNX model"
)
parser.add_argument("onnx", metavar="FILE", help="path to onnx file")
parser.add_argument(
    "-k",
    "--keepTopK",
    metavar="FILE",
    help="number of bounding boxes for nms",
    default=1,
)
parser.add_argument("-t", "--topK", help="number of bounding boxes for nms", default=500)
parser.add_argument(
    "-s", "--score", help="number of bounding boxes for nms", default=0.4
)
parser.add_argument("-i", "--iou", help="number of bounding boxes for nms", default=0.6)
parser.add_argument("--nbCls", help="number of classes", default=6)

if __name__ == "__main__":
    args = parser.parse_args()
    onnxPath = Path(args.onnx)

    if not onnxPath.exists():
        raise FileNotFoundError("Model not exists")
    if "nms" in str(onnxPath):
        print(r"File name already has 'nms', do nothing")
        exit(0)

    graph = gs.import_onnx(onnx.load(onnxPath))

    graph = create_and_add_plugin_node(graph, args)
    print(f"Insert NMS layer to model {onnxPath} successfully")

    outPath = onnxPath.with_name(str(onnxPath.stem) + "_nms.onnx")
    onnx.save(gs.export_onnx(graph), outPath)
    print(f"Saved new model")
