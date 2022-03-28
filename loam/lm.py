"""
# Implement LM algorithm only using basic python
# Author:Leo Ma
# For csmath2019 assignment4,ZheJiang University
# Date:2019.04.28
# Original sources: 
#     * https://pythonmana.com/2020/12/20201210164251696e.html
#     * http://users.ics.forth.gr/~lourakis/levmar/levmar.pdf
# Changes since original:
#   * replaced derivative calculation
#   * Fixes visualization of parameters and its estimations
#   * clean main algorithm loop
#   * remove unused functions
"""
import numpy as np
import matplotlib.pyplot as plt

tao = 10**-3
threshold_stop = 10**-15
threshold_step = 10**-15
threshold_residuals = 10**-15
residuals_memory = []

# construct a user function
def function(params, input_data):
    a = params[0, 0]
    b = params[1, 0]
    return a * np.exp(b * input_data)


# generating the input_data and output_data,whose shape both is (num_data,1)
def generate_data(params, num_data):
    x = np.array(np.linspace(0, 10, num_data)).reshape(num_data, 1)  
    
    # Generating data that contains noise
    mid, sigma = 0, 5
    y = function(params, x) + np.random.normal(mid, sigma, num_data).reshape(num_data, 1)
    return x, y


# calculating the derivative of pointed parameter,whose shape is (num_data,1)
# f'(x) = (f(x + h) - f(x)) / h
def derivative(params, input_data, param_index):
    h = 0.00000000001

    params1 = params.copy()
    params1[param_index, 0] += h
    params2 = params.copy()

    top = function(params1, input_data) - function(params2, input_data)
    bottom = h
    slope = top / bottom
    return slope


# calculating j matrix,whose shape is (num_data,n_params)
def jacobian(params, input_data):
    n_params = np.shape(params)[0]
    num_data = np.shape(input_data)[0]
    J = np.zeros((num_data, n_params))

    for i in range(n_params):
        J[:, i] = list(derivative(params, input_data, i))

    return J


# calculating residuals, whose shape is (num_data,1)
def residuals(params, input_data, output_data):
    data_est_output = function(params, input_data)
    residuals = output_data - data_est_output
    return residuals


def get_init_λ(A, tao):
    m = np.shape(A)[0]
    Aii = []
    for i in range(0, m):
        Aii.append(A[i, i])
    u = tao * max(Aii)
    return u


# get the init u, using equation u=tao*max(Aii)
def get_init_u(A, tao):
    m = np.shape(A)[0]
    Aii = []
    for i in range(0, m):
        Aii.append(A[i, i])
    u = tao * max(Aii)
    return u


# LM algorithm as presented in http://users.ics.forth.gr/~lourakis/levmar/levmar.pdf
# variable names match those in paper
# 
# They relate to usual syntax like this
#   λ ~= u
#   H ~= A
#
def LM(max_iter, params, input_data, output_data):
    n_params = np.shape(params)[0]  # the number of params
    iter = 0  # set the init iter count is 0

    residual = residuals(params, input_data, output_data) # init residual
    J = jacobian(params, input_data) # init J matrix
    A = J.T.dot(J)  # calculating the init A
    g = J.T.dot(residual)  # calculating the init gradient g

    stop = np.linalg.norm(g, ord=np.inf) <= threshold_stop  # set the init stop
    u = get_init_u(A, tao)  # set the init u
    v = 2  # set the init v=2

    while (not stop) and (iter < max_iter):
        iter += 1
        while 1:
            step = np.linalg.inv(A + u * np.eye(n_params)).dot(g)  # calculating the update step
            
            if np.linalg.norm(step) <= threshold_step:
                stop = True
            else:
                new_params = params + step  # update params using step
            
            new_residual = residuals(new_params, input_data, output_data) # get new residual using new params
            rou = (np.linalg.norm(residual) ** 2 - np.linalg.norm(new_residual) ** 2) / (
                step.T.dot(u * step + g)
            )
            if rou > 0:
                params = new_params
                residual = new_residual
                residuals_memory.append(np.linalg.norm(residual) ** 2)
                
                J = jacobian(params, input_data) # recalculating J matrix with new params
                A = J.T.dot(J)  # recalculating A
                g = J.T.dot(residual)  # recalculating gradient g
                stop = (np.linalg.norm(g, ord=np.inf) <= threshold_stop) or (
                    np.linalg.norm(residual) ** 2 <= threshold_residuals
                )
                u = u * max(1 / 3, 1 - (2 * rou - 1) ** 3)
                v = 2
            else:
                u = u * v
                v = 2 * v

            if rou > 0 or stop:
                break

    return params


def main():
    # set the true params for generate_data() function
    params = np.zeros((2, 1))
    params[0, 0] = 10.0
    params[1, 0] = 0.8
    num_data = 100  # set the data number
    data_input, data_output = generate_data(params, num_data)  # generate data as requested
    print("real params:\n ", params)
    # set the init params for LM algorithm
    params[0, 0] = 6.0
    params[1, 0] = 0.3

    # using LM algorithm estimate params
    max_iter = 100
    est_params = LM(max_iter, params, data_input, data_output)
    print("\nestimated\n", est_params)
    a_est = est_params[0, 0]
    b_est = est_params[1, 0]

    # generate data plot
    plt.scatter(data_input, data_output, color="b")

    # Generate result plot
    x = np.arange(0, 100) * 0.1
    plt.plot(x, a_est * np.exp(b_est * x), "r", lw=1.0)
    plt.savefig("result_LM.png")
    plt.show()
    plt.plot(residuals_memory)
    plt.xlabel("iteration")
    plt.ylabel("error")
    plt.savefig("error-iter.png")
    plt.show()


if __name__ == "__main__":
    main()
