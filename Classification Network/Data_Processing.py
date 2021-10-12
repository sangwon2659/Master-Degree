import numpy as np
import math
import pandas as pd

# Number of lookbacks for the covariance computation
n_covariance = 10
# Number of sample frequencies required
n_frequency = 10
# Scaling the covariance in order to match significance with the frequency data
scale_covariance = 1000
normalizer_constant = 1

n_sample_frequency = n_frequency*2
FSS = np.loadtxt("Dataset.csv", delimiter=",")
FSS_sum_array = np.array((0.0))
FSS_previous = FSS[n_sample_frequency-1]
Diff_Matrix = np.zeros((n_covariance, n_covariance))
Combined_Vector = np.zeros(n_sample_frequency/2+1)
Combined_Vector_Sum = np.zeros(n_sample_frequency/2+1)

for i in range(np.shape(FSS)[0]):
    FSS_sum = FSS[i].sum()/normalizer_constant
    FSS_sum_array = np.append(FSS_sum_array, FSS_sum)

    if len(FSS_sum_array) > n_sample_frequency:
        FSS_sum_array = np.delete(FSS_sum_array, [0])

        ### Covariance
        # Computing the difference between the latest and second lastest
        FSS_diff = FSS[i] - FSS_previous
        # Inserting the data into Difference Matrix queue
        Diff_Matrix = np.vstack((FSS_diff, Diff_Matrix))
        # Deleting the earliest FSS_diff
        Diff_Matrix = np.delete(Diff_Matrix, n_covariance, axis=0)
        # Updating FSS_previous with the latest FSS data
        FSS_previous[:] = FSS[i][:]

        # Computing the covariance
        Cov_Matrix = np.dot(np.transpose(Diff_Matrix), Diff_Matrix)
        # Inserting the covariance value into the last position of Slip Vector array
        Covariance = (np.sum(Cov_Matrix) - np.trace(Cov_Matrix)) * scale_covariance

        ### Frequency
        FFT = np.fft.fft(FSS_sum_array)
        # First Element of the FFT array should be included or not?
        # 0~(n_sample_frequency/2)-1 -> Positive region values
        # n_sample_frequency/2~n_sample_frequency -> Negative region values
        FFT = abs(FFT[0:n_sample_frequency/2])

        Combined_Vector[0:n_sample_frequency/2] = FFT
        Combined_Vector[n_sample_frequency/2] = Covariance

        Combined_Vector_Sum = np.vstack((Combined_Vector_Sum, Combined_Vector))

df = pd.DataFrame(Combined_Vector_Sum)
df.to_csv('Processed_Data_10Hz.csv')
