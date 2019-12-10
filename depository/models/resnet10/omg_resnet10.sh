#!/bin/bash
DDK_PATH=/your/ddk/path/
omg_bin=$DDK_PATH/bin/x86_64-linux-gcc5.4/omg
$omg_bin \
--model=./resnet10.pb \
--input_shape="image_tensor:1,112,112,3" \
--framework=3 --output=./resnet10_b1 \
--insert_op_conf=./aipp_resnet10.config;

$omg_bin \
--model=./resnet10.pb \
--input_shape="image_tensor:4,112,112,3" \
--framework=3 --output=./resnet10_b4 \
--insert_op_conf=./aipp_resnet10.config;

$omg_bin \
--model=./resnet10.pb \
--input_shape="image_tensor:8,112,112,3" \
--framework=3 --output=./resnet10_b8 \
--insert_op_conf=./aipp_resnet10.config
