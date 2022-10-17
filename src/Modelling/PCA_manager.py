import numpy as np

class PCA_manager(object):
    def __init__(self):
        self.data_queue = []
        self.num_data_points = 0

        self.sum_X = None
        self.sum_XXt = None
        self.covariance = None

    def push(self,X):
        X=np.array([X]).T

        if X.ndim != 2:
            print('error: input vector should only be 1D, input refused')
            return

        self.data_queue.append(X)
        self.num_data_points+=1

        if self.sum_X is None:
            self.sum_X=X+0.0
        else:
            self.sum_X+=X

        if self.sum_XXt is None:
            self.sum_XXt = np.dot(X,X.T)
        else:
            self.sum_XXt+=np.dot(X,X.T)


    def pop(self):
        if len(data_queue)==0:
            print('error: no data in queue, cannot pop')
            return

        X=self.data_queue.pop(0)

        self.num_data_points-=1
        self.sum_X-=X
        self.sum_XXt-=np.dot(X,X.T)

    def compute_PCA(self):
        if len(data_queue)==0:
            print('error: no data in queue, cannot compute anything')
            return None,None

        self.compute_covariance()
        return np.linalg.eigh(self.covariance)

    def compute_covariance(self):

        X_mean = self.sum_X/self.num_data_points

        self.covariance = (self.sum_XXt/self.num_data_points)-np.dot(X_mean,X_mean.T)



