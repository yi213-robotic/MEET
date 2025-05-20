# Meet-in-the-middle with Early and Efficient Termination (MEET)
## Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman
In bidirectional heuristic search, the meeting-in-the-middle property (MMP) and the theory of must-expand pairs (MEP) have driven significant recent developments in search efficiency. However, these methodologies typically terminate the search based on minimal priority metrics in the forward and backward open lists, requiring exploration of all potentially better solutions and potentially incurring substantial computational burden. In this paper, we investigate the reasons that contribute to the potential inefficiency in MM , and introduce a tighter termination condition that enables earlier termination without exhaustive exploration while still ensuring both MMP and optimality. This results in a highly efficient bidirectional search algorithm.
Experimental comparisons demonstrate that our algorithm outperforms MM in terms of running time by at least two orders of magnitude and is on par or better compared to A* , highlighting its potential in a wide range of applications. 

<img width="1101" alt="meet_IJCAI" src="https://github.com/user-attachments/assets/cc294713-2961-4875-92e4-9261759e23a9" />

<img width="1115" alt="State Expansion" src="https://github.com/user-attachments/assets/20494aec-1627-4d01-b6d3-51151a023993" />

<img width="1198" alt="Reduction" src="https://github.com/user-attachments/assets/bfabaedf-07a4-43c0-8e88-98cac42973a7" />


[MEET_appendix_.pdf](https://github.com/user-attachments/files/20354553/MEET_appendix_.pdf)


🚀 **[MEET has been accepted to the 34th International Joint Conference on Artificial Intelligence (IJCAI 2025)](https://2025.ijcai.org)!** The C++ code will be released after the conference.

