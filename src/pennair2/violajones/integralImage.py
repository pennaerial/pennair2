import numpy as np

# In an integral image each pixel is the sum of all pixels
# in the original image that are 'left and above' the pixel.
def toIntegralImg(doubleArr):
	# last row/column gets -1; rowSum(x, -1) == 0 holds for all x
    rowSum = np.zeros(doubleArr.shape)
    intArr = np.zeros((doubleArr.shape[0] + 1, doubleArr.shape[1] + 1))
    for x in range(doubleArr.shape[1]):
        for y in range(doubleArr.shape[0]):
            rowSum[y, x] = rowSum[y-1, x] + doubleArr[y, x]
            intArr[y+1, x+1] = intArr[y+1, x-1+1] + rowSum[y, x]
    return intArr

# takes top left (x, y) and bottom right (x, y) to compute sum
# d---b
# |	  |
# c---a
def sumRegion(intArr, TLTup, BRTup):
	d = (d[1], d[0])
    a = (a[1], a[0])
    if d == a:
        return intArr[d]
    b = (a[0], d[1])
    c = (d[0], a[1])
    return intArr[a] - intArr[b] - intArr[c] + intArr[d]
