#!/bin/bash

echo "视频设备与USB端口映射:"
echo "------------------------"

for dev in /dev/video*; do
  # 获取设备基本信息
  dev_name=$(basename $dev)
  dev_node=$dev
  
  # 获取USB端口路径
  usb_path=$(udevadm info --query=property --name=$dev | grep "ID_PATH=" | cut -d= -f2)
  
  # 获取物理USB端口
  physical_port=$(udevadm info --query=path --name=$dev | grep -o "[0-9]-[0-9]\([.-][0-9]\)*")
  
  # 获取制造商和产品信息
  vendor_product=$(udevadm info --query=property --name=$dev | grep -E "ID_VENDOR_ID|ID_MODEL_ID" | cut -d= -f2 | tr '\n' ':' | sed 's/:$//')
  
  # 获取设备名称
  device_name=$(v4l2-ctl --info --device=$dev 2>/dev/null | grep "Card type" | cut -d: -f2 | sed 's/^\s*//')
  
  echo "设备: $dev_node"
  echo "名称: ${device_name:-未知}"
  echo "USB路径: $usb_path"
  echo "物理端口: $physical_port"
  echo "产品ID:厂商: $vendor_product"
  echo "------------------------"
done
