import simulation as sim
import numpy as np

from Environment import Environment

from SAC import sac_trainer
from torch.utils.tensorboard import SummaryWriter
from init import action_dim,state_dim


DETERMINISTIC=False

model_path = './model/SAC_1_27'

env = Environment()


if __name__ == "__main__":

    sac_trainer.load_model(model_path)
    #Tesnorboard
    writer = SummaryWriter('./test_log')

    avg_reward = 0.
    episodes = 5
    cal_dist_set = []
    for _  in range(episodes):
        
        state = env.reset()
        episode_reward = 0
        done = False

        for i in range(1000):
            action = sac_trainer.policy_net.get_action(state, deterministic = DETERMINISTIC)

            next_state, reward, done = env.step(action)
            episode_reward += reward
                
            state = next_state
            if done:                         ## 实际中不可能reset包裹,暂时支持reset！！！！！！！！！！
                # state = env.reset()
                pass
                
            if env.evl_flag == 1:       ## 有包裹过线
                # delta_T_set.append(env.delta_t)
                cal_dist_set.append(env.cal_dist)
                env.evl_flag = 0

        avg_reward += episode_reward
    episode_reward = 0
    avg_reward /= episodes

    # delta_T_set = np.array(delta_T_set)
    cal_dist_set = np.array(cal_dist_set)
    writer.add_scalar('avg_reward/test', avg_reward, 1)

    writer.add_histogram('delta_dist', cal_dist_set, 1, bins = 500)         # 看delta T是否收敛到设定值

    print("----------------------------------------")
    print("Test Episodes: {}, Avg. Reward: {}, Avg. delta_dist: {}".format(episodes, round(avg_reward, 2),np.mean(cal_dist_set)))
    print("----------------------------------------")