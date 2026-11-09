warehouse-10-20-10-2-1, 20 agents: both planners solved 23 of 25 scenarios (1-25, time limit 5 s); 20 delay seeds per plan and probability
following moves per plan (a robot entering a cell in the step another leaves it): pp 24.4, cbs 7.3

| planner | p | runs | timetable: runs with a collision | collisions per run | ordered: delay robot-steps per run | extra SOC per delay | extra makespan per run |
|---|---:|---:|---:|---:|---:|---:|---:|
| pp | 0.01 | 460 | 53.5% | 5.50 | 15.96 | 1.050 | 1.87 |
| pp | 0.05 | 460 | 76.1% | 5.71 | 83.39 | 1.038 | 9.17 |
| pp | 0.1 | 460 | 85.0% | 5.85 | 175.67 | 1.033 | 19.47 |
| pp | 0.2 | 460 | 85.9% | 6.33 | 395.90 | 1.024 | 43.01 |
| cbs | 0.01 | 460 | 23.5% | 1.42 | 15.44 | 1.022 | 1.86 |
| cbs | 0.05 | 460 | 50.4% | 2.43 | 80.92 | 1.020 | 9.14 |
| cbs | 0.1 | 460 | 65.0% | 3.12 | 170.42 | 1.018 | 19.39 |
| cbs | 0.2 | 460 | 69.6% | 3.97 | 383.41 | 1.014 | 42.98 |

| planner | delay | runs | timetable: runs with a collision | ordered: extra SOC per run | of it on other robots | other robots later |
|---|---:|---:|---:|---:|---:|---:|
| pp | 1 | 460 | 6.7% | 1.033 | 0.072 | 0.072 |
| pp | 3 | 460 | 8.3% | 3.254 | 0.293 | 0.117 |
| pp | 5 | 460 | 9.8% | 5.543 | 0.583 | 0.157 |
| cbs | 1 | 460 | 1.3% | 1.013 | 0.013 | 0.013 |
| cbs | 3 | 460 | 3.0% | 3.100 | 0.100 | 0.050 |
| cbs | 5 | 460 | 5.2% | 5.243 | 0.243 | 0.085 |
