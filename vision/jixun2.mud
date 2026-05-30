[basic]
type = axmodel
model_npu = jixun2_npu.axmodel
model_vnpu = jixun2_vnpu.axmodel

[extra]
model_type = yolo11
type = detector
input_type = rgb
labels = apple,banana,strawberry,watermelon,target,green_square,red_triangle,yellow_trapezoid
input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true

mean = 0,0,0
scale = 0.00392156862745098,0.00392156862745098,0.00392156862745098
