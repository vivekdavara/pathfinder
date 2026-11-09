random-32-32-20, 20 agents: both planners solved 24 of 25 scenarios (1-25, time limit 5 s); 20 delay seeds per plan and probability
following moves per plan (a robot entering a cell in the step another leaves it): pp 18.8, cbs 14.8

| planner | p | runs | timetable: runs with a collision | collisions per run | ordered: delay robot-steps per run | extra SOC per delay | extra makespan per run |
|---|---:|---:|---:|---:|---:|---:|---:|
| pp | 0.01 | 480 | 57.3% | 2.03 | 4.74 | 1.205 | 0.68 |
| pp | 0.05 | 480 | 91.9% | 5.83 | 25.90 | 1.153 | 2.99 |
| pp | 0.1 | 480 | 96.7% | 7.28 | 53.26 | 1.151 | 6.06 |
| pp | 0.2 | 480 | 98.1% | 8.98 | 121.14 | 1.112 | 12.90 |
| cbs | 0.01 | 480 | 42.3% | 1.39 | 4.55 | 1.142 | 0.67 |
| cbs | 0.05 | 480 | 82.7% | 4.26 | 24.72 | 1.121 | 3.02 |
| cbs | 0.1 | 480 | 92.7% | 5.83 | 50.86 | 1.123 | 6.19 |
| cbs | 0.2 | 480 | 97.9% | 7.20 | 115.11 | 1.094 | 13.12 |

| planner | delay | runs | timetable: runs with a collision | ordered: extra SOC per run | of it on other robots | other robots later |
|---|---:|---:|---:|---:|---:|---:|
| pp | 1 | 480 | 20.6% | 1.227 | 0.277 | 0.277 |
| pp | 3 | 480 | 29.4% | 4.219 | 1.271 | 0.567 |
| pp | 5 | 480 | 34.4% | 7.688 | 2.740 | 0.785 |
| cbs | 1 | 480 | 16.2% | 1.194 | 0.206 | 0.206 |
| cbs | 3 | 480 | 25.6% | 4.019 | 1.031 | 0.479 |
| cbs | 5 | 480 | 27.9% | 7.360 | 2.373 | 0.713 |
