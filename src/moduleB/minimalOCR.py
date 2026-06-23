import easyocr

reader = easyocr.Reader(["ch_sim", "en"])
result = reader.readtext("./samples/test.png", detail=0)
#如果不加detail=0，则result是一个列表，每个元素是一个元组，元组中第一个元素是文本，第二个元素是文本的坐标
# 后面这一大串列表就是 reader.readtext(...) 的原始返回值。EasyOCR 默认 detail=1，所以每个识别结果都是一个三元组：

# (
#   文本框四个角的坐标,
#   识别出来的文字,
#   置信度
# )

print(result)