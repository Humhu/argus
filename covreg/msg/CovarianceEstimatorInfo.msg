# Message defining covariance estimator parameters

# The source this covariance estimator corresponds to
string source_name

# The correlation parameters
float64[] correlation_parameters

# The variance estimator parameters
NeuralNetworkInfo variance_info

# The output offset
argus_msgs/MatrixFloat64 offset