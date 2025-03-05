#!/bin/bash
#SBATCH --job-name=cpu_quad_sim
#SBATCH --nodes=1
#SBATCH --ntasks=8   # Use more tasks to parallelize
#SBATCH --cpus-per-task=4
#SBATCH --time=02:00:00
#SBATCH --output=cpu_simulation_output.log

# Load required modules
module load python/3.8
module load mpi
module load pybullet

# Run simulation using MPI for parallel processing
mpirun -np 8 python quad.py