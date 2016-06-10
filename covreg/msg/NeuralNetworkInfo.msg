# Message defining configuration and parameters of a neural network

# The type of activation to use in this network
# Can be ['sigmoid', 'leaky_relu']
string activation_type

# Parameters for the activation type
float64[] activation_params

# Dimension N of the output N x N matrix
uint32 output_dim

# Dimension of the input vector
uint32 input_dim

# Number of 'hidden' layers
# Number of actual layers is num_hidden_layers + 1
uint32 num_hidden_layers

# Width of the hidden layers
uint32 layer_width

# The network parameters in vector form
float64[] parameters