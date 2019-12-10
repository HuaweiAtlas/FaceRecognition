#!/bin/bash
DDK_PATH=/your/ddk/path/
omg_bin=$DDK_PATH/bin/x86_64-linux-gcc5.4/omg
$omg_bin \
--model=./yolov3tiny_b4.prototxt \
--weight=./yolov3tiny.caffemodel \
--framework=0 --output=./yolov3tiny_b4 \
--insert_op_conf=./aipp_yolov3tiny.config 
