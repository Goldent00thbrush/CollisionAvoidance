import numpy
def GetEnvLines(env):
    Lines = numpy.zeros(len(env.dx_dy), 4)

    intial_point = env.intial_point
    for i in range(1,len(env.dx_dy)):
        new_point = intial_point
        if (i% 2 == 1):
            new_point[1] = new_point[1] + env.dx_dy(i)
        else:
            new_point[2] = new_point[2] + env.dx_dy(i)
        Lines[i,:] = [intial_point[1],intial_point[2],new_point[1],new_point[2]]
        intial_point = new_point
    return Lines