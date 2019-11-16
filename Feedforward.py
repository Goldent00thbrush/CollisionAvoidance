import numpy
def Feedforward(Sample, Chromosome, Network_Arch, unipolarBipolarSelector):
    #Feed Forward
    activations = [Sample, 1] # Adding Bias Node
    startId = 0
    for Layer in range( 2,len(Network_Arch)):
        d1 = len(activations)
        d2 = Network_Arch(Layer)
        weights = Chromosome[startId + 1: startId + d1 * d2]
        weigths = numpy.reshape(weights, d1, d2)
        activations = activations * weigths
        if (unipolarBipolarSelector == 0):
            activations = 1. / (1 + numpy.exp(-activations))
        else:
            activations = -1 + 2. / (1 + numpy.exp(-activations))
        if (Layer != len(Network_Arch)): # Adding Bias
            activations = [activations, 1]
        startId = d1 * d2

    outputs = activations

    return outputs