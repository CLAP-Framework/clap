# Describing the size object in 3D space (in meters) with uncertainty

float64 width
float64 height
float64 length

# Row-major representation of the 3x3 covariance matrix
# In order, the parameters are: (width, height, length)
float64[9] covariance