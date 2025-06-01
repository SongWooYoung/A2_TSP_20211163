# A2_TSP_20211163

1. christofides is now on implementing

2. vcg_mst_tsp.cpp is a c++ version of vcg_mst_tsp, which is made for calculating the 
   speed and memory usage. Because python is too slow to measure. 
   it takes 20 minutes to complete and due to grid search you have to modify some codes, otherwise, it would take long time to meet result.

3. to test each algorithm, you have to put the file name on test3 and control 
   the max value, which is because held karp is not working more than 23 samples...
   YOU MUST CHANGE  " if (++count >= 100000) break" this part. it is okay to excess max number of data.

