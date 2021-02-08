## state space
Parcel_number = 3*3     #预设输入网络的最大包裹数
state_per_dim = 5
state_dim = Parcel_number * state_per_dim

## action space
col_num = 5
row_num = 3*2
action_dim = (col_num - 1)*row_num + 1 ## 最后一列的无法控制