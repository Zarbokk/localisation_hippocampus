from particle_class import ParticleFilter
import numpy as np
pf = ParticleFilter(20, 3, (0, 5), (0, 5), (0, 5))
pf.update(np.asarray([[3, 1, 1, 0], [4, 0, 0, 0]]))
