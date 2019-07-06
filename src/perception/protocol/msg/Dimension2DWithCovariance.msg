# Describing the size object in 2D space (in pixels) with uncertainty

float64 width
float64 length

# Row-major representation of the 2x2 covariance matrix
# In order, the parameters are: (width, length)
float64[4] covariance
