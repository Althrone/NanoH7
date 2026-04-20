import os
import struct

result = os.system('arm-none-eabi-readelf -a S32K3x4_Code.FLM >elf_prase.log')
result = os.system('arm-none-eabi-objdump -s -d S32K3x4_Code.FLM >dis_asm.log')

PrgCode_file=open('PrgCode','r')
PrgCode_content=PrgCode_file.read()
# print(PrgCode_content)
# byte_data = bytes.fromhex(PrgCode_content)
PrgCode_strings = PrgCode_content.replace("\n", " ").split()
# print(PrgCode_strings)
uint32_array = []
for hex_str in PrgCode_strings:
    # 将十六进制字符串转换为字节
    byte_data = bytes.fromhex(hex_str)
    # 将字节解包为 uint32（这里假设小端字节序，如果需要大端，请使用 '>I'）
    # 根据你的具体需求，确定端序。这里我们假设小端（常见的 x86 架构默认小端）
    # 如果需要大端，请使用 '>I'
    if len(byte_data) == 4:
        uint32_value = struct.unpack('<I', byte_data)[0]
        uint32_array.append(uint32_value)
    else:
        print(f"警告：十六进制字符串 {hex_str} 长度不是 8 位，跳过")
# print(uint32_array)
f=open('c_PrgCode.c', 'w')
f.write('static const uint32 PrgCode[]={\n    ')
changeline=0
for output in uint32_array:
    temp2=f"0x{output:08x}"
    f.write(temp2)
    f.write(',')
    changeline+=1
    if(changeline==4):
        changeline=0
        f.write('\n    ')
f.write('};')
#找到 Contents of section PrgCode: Contents of section PrgData: Contents of section DevDscr: 所在行数

file_path='dis_asm.log'
first_match_str='Contents of section'
PrgCode_str='Contents of section PrgCode:'
PrgData_str='Contents of section PrgData:'
DevDscr_str='Contents of section DevDscr:'

# with open(file_path, 'r', encoding='utf-8') as file:
#     for line_num, line in enumerate(file, start=1):
#         if first_match_str in line:
#             if(line_num>PrgCode_line_start):
#                 PrgCode_line_end=line_num-1
#             if PrgCode_str in line:
#                 PrgCode_line_start=line_num+1

# print(PrgCode_line_start,PrgCode_line_end)
# with open(file_path, 'r', encoding='utf-8') as file:
#     for line_num, line in enumerate(file, start=1):
#         if PrgData_str in line:
#             PrgData_line=line_num

# with open(file_path, 'r', encoding='utf-8') as file:
#     for line_num, line in enumerate(file, start=1):
#         if DevDscr_str in line:
#             DevDscr_line=line_num

# PrgCode_line+=1
# PrgData_line+=1
# DevDscr_line+=1

# print(PrgCode_line,PrgData_line,DevDscr_line)