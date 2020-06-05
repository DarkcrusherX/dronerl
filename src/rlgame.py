import tensorflow as tf
import rospy
import numpy as np
import base64, io, time, gym
import IPython, functools
import matplotlib.pyplot as plt
from tqdm import tqdm
import mitdeeplearning as mdl
import myquadcopter_env
rospy.init_node('rl', anonymous=True)
### Define the snake agent ###
Dense = tf.keras.layers.Dense
rate = rospy.Rate(10)
# Defines a CNN for the Pong agent
def quad_model():
	model = tf.keras.models.Sequential([
		# Fully connected layer and output
		Dense(units = 15,  kernel_initializer = 'uniform', activation = 'relu', input_dim = 4),
		Dense(units = 10,  kernel_initializer = 'uniform', activation = 'relu', input_dim = 15),
		Dense(units = 6,  kernel_initializer = 'uniform', activation = 'relu', input_dim = 10)

  
    ])
	return model

def choose_action(model, observation,n_actions):
	# # add batch dimension to the observation
	observation = np.expand_dims(observation, axis=0)

	logits = model.predict(observation) # TODO
  # logits = model.predict('''TODO''')

  # pass the log probabilities through a softmax to compute true probabilities
	prob_weights = tf.nn.softmax(logits).numpy()

	action = np.random.choice(6, size=1, p=prob_weights.flatten())[0] # TODO
  # action = np.random.choice('''TODO''', size=1, p=''''TODO''')['''TODO''']
	rate.sleep()
	return action

### Agent Memory ###

class Memory:
	def __init__(self): 
		self.clear()

  # Resets/restarts the memory buffer
	def clear(self): 
		self.observations = []
		self.actions = []
		self.rewards = []

  # Add observations, actions, rewards to memory
	def add_to_memory(self, new_observation, new_action, new_reward): 
		self.observations.append(new_observation)
		self.actions.append(new_action) # TODO
		self.rewards.append(new_reward) # TODO


def normalize(x):
	x -= np.mean(x)
	x /= np.std(x)
	return x.astype(np.float32)  

def discount_rewards(rewards, gamma=0.99): 
	discounted_rewards = np.zeros_like(rewards)
	R = 0
	for t in reversed(range(0, len(rewards))):
		# NEW: Reset the sum if the reward is not 0 (the game has ended!)
		if rewards[t]!=0:
	  		R = 0
		# update the total discounted reward as before
		R = R * gamma + rewards[t]
		discounted_rewards[t] = R
	  
	return normalize(discounted_rewards)

def compute_loss(logits, actions, rewards): 

	neg_logprob = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=logits, labels=actions) # TODO
  # neg_logprob = tf.nn.sparse_softmax_cross_entropy_with_logits(logits='''TODO''', labels='''TODO''')
  

	loss = tf.reduce_mean( neg_logprob * rewards ) # TODO
  # loss = tf.reduce_mean('''TODO''')
	return loss

def train_step(model, optimizer, observations, actions, discounted_rewards):
	with tf.GradientTape() as tape:
	  # Forward propagate through the agent network
		logits = model(observations)
		loss = compute_loss(logits, actions, discounted_rewards) # TODO
	  # loss = compute_loss('''TODO''', '''TODO''', '''TODO''')
	grads = tape.gradient(loss, model.trainable_variables) # TODO
  # grads = tape.gradient('''TODO''', model.trainable_variables)
	optimizer.apply_gradients(zip(grads, model.trainable_variables)) 




if __name__ == '__main__':


	# Create the Gym environment
	env = gym.make('QuadcopterLiveShow-v0')
	n_actions = env.action_space.n

	# Hyperparameters
	learning_rate=1e-4
	MAX_ITERS = 10000 # increase the maximum number of episodes

	# Model and optimizer
	model = quad_model()
	optimizer = tf.keras.optimizers.Adam(learning_rate)

	# plotting
	smoothed_reward = mdl.util.LossHistory(smoothing_factor=0.9)
	plotter = mdl.util.PeriodicPlotter(sec=5, xlabel='Iterations', ylabel='Rewards')
	memory = Memory()

	for i_episode in range(MAX_ITERS):
		print(" ")
		print("EPISODE : {}".format(i_episode))
		print(" ")

		# if i_episode != 1 :
		# 	model.summary(line_length=None, positions=None, print_fn=None)
		# 	print(model.trainable_variables)
		
		plotter.plot(smoothed_reward.get())

		# Restart the environment
		observation = env.reset()
		#previous_frame = mdl.lab3.preprocess_pong(observation)
		original_observation = observation

		while True:
			# Pre-process image 
			#current_frame = mdl.lab3.preprocess_pong(observation)
	  
			obs_change = abs(observation - original_observation) # TODO
			# obs_change = # TODO
	  
			action = choose_action(model, obs_change,n_actions) # TODO 
			# action = # TODO
			# Take the chosen action
			next_observation, reward, done, info = env.step(action)

			memory.add_to_memory(obs_change, action, reward) # TODO
	  

			if done:
				# determine total reward and keep a record of this
				total_reward = sum(memory.rewards)
				smoothed_reward.append( total_reward )
				print(" ")
				print("REWARD : {}".format(total_reward))
				print(" ")

				# begin training
				train_step(model, optimizer, observations = np.stack(memory.observations, 0), actions = np.array(memory.actions),discounted_rewards = discount_rewards(memory.rewards))
		  
				memory.clear()
				break

			observation = next_observation
			#previous_frame = current_frame
