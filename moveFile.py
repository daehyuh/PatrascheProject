import shutil
import os

f = open("path.txt", 'r', encoding='UTF8' ) # path.txt 에서 옮길 파일이름 지정
line = f.readline()

file_path = line
move_path = 'move_data'
for dir in os.listdir(file_path):
    print(dir)
    shutil.move(os.path.join(file_path,dir), os.path.join(file_path, dir))
sys.exit(0)