import numpy as np
import pbdlib as pbd
from sklearn.base import BaseEstimator

class GMMR(BaseEstimator):
    def __init__(self, nb_states=5, reg=1e-8):
        self.nb_states = nb_states
        self.reg = reg
        return

    def fit(self, data):
        data_start = []
        data_end = []
        reg = self.reg # There can also be unique reg's for each dimension

        # Transform data to put start point at origin
        for p in data:
            # Set start point
            start = p[0, :]
            arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, start)
            data_start.append(arr_trans)

        # Transform data to put end point at origin
        for p in data:
            # Set end point
            end = p[-1, :]
            arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, end)
            data_end.append(arr_trans)

        # Define time axis
        self.t = np.linspace(0, 100, data[0].shape[0])

        # Create list of arrays for time, start point, and end point TODO: check
        gmm_demos = [np.hstack([self.t[:, None], s, e]) for e in data_end for s in data_start]
        # Create array with all poses in demonstrations
        gmm_demos2 = np.vstack([d for d in gmm_demos])

        # Initialize GMM
        self.gmm_ = pbd.GMM(nb_dim=13, nb_states=self.nb_states)

        # Initialize gaussians over data
        self.gmm_.init_hmm_kbins(gmm_demos)

        # Train model
        self.gmm_.em(gmm_demos2, reg=reg)

        return self

    def predict(self, data):
        try:
            getattr(self, "gmm_")
        except AttributeError:
            raise RuntimeError("You must train classifer before predicting data!")

        repros = []

        for i in range(len(data)):
            # Create arrays for transforming data
            A0 = np.identity(n=7)
            An = np.identity(n=7)
            b0 = np.zeros(7)
            bn = np.zeros(7)
            A0[1:7, 1:7], b0[1:7] = pbd.utils.inv_for_lintrans(data[i][0, :])
            An[1:7, 1:7], bn[1:7] = pbd.utils.inv_for_lintrans(data[i][-1, :])
            
            # Select columns for split models
            dim1 = np.array([0, 1, 2, 3, 4,  5,  6])
            dim2 = np.array([0, 7, 8, 9, 10, 11, 12])
            
            # Split models
            _mod1 = self.gmm_.marginal_array(dim1).lintrans(A0, b0)
            _mod2 = self.gmm_.marginal_array(dim2).lintrans(An, bn)

            # Combine models
            _prod = _mod1 * _mod2

            # Get the most probable trajectory for this initial and final pose
            _mu, _sigma = _prod.condition(self.t[:, None], dim_in=slice(0, 1), dim_out=slice(0, 7))

            repros.append(_mu[:, 1:])

        return repros

    def score(self, demo):
        costsum = 0

        repro = self.predict(demo)
        for i in range(len(repro)):
            ## Cost function
            # Initial pose
            diff_init = sum(((repro[i][0, :] - demo[i][0])*np.array([10,5,20,20,20,10]))**2)

            # Final pose
            diff_end = sum(((repro[i][-1, :] - demo[i][-1])*np.array([10,5,20,20,20,10]))**2)

            # Trajectory length
            diff_repro = np.diff(repro[i], axis=0)
            path_length = sum(np.apply_along_axis(pbd.utils.pythagoras_3d, 1, diff_repro))*6

            # Initial trajectory
            start_repro = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, repro[i], demo[i][0])
            row_index = np.argmax(start_repro[:, 2] > 0.025)
            diff_traj = (sum(((start_repro[row_index, [0, 1, 3, 4]])*np.array([10,5,20,20]))**2))*1.5

            # Sum costs
            costsum -= diff_init
            costsum -= diff_end
            costsum -= path_length
            costsum -= diff_traj

        return costsum / len(demo)

