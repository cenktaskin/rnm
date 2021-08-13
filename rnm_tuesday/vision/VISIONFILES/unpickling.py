import pickle
import cv2
import numpy as np

images = 0

with open("D:/Master tuhh 2020/Sem2/RNM files/scanning_output (1)/scanning_output/scanning.pkl", "rb") as fp:   # Unpickling
    b = pickle.load(fp)

with open('D:/Master tuhh 2020/Sem2/RNM files/www/www/tf3.txt', 'w') as f:
    for i in b:
       # print("image")
        #print(i[0])
       # print("tf")
       # print(i[1])
       # cv2.imshow('im',i[0])
        #cv2.imwrite("D:/Master tuhh 2020/Sem2/RNM files/www/www/unpickleImages/"+str(images)+ ".jpg", i[0])
        #images = images+1

        mat = np.asarray(i[1])
        print(mat)
        mat = mat.flatten()
        lines = []

        for j in range(len(mat)):
            a = mat[j]
            b = str(a)
            print("str",b)
            lines.append(b)

        for line in lines:
            f.write(line)
            f.write(',')
        f.write('\n')
print("Writing done...!!!")
    