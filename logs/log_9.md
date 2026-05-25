## **In this session, I worked during 7 hours**
<br> <br>

## I modified "main.py" to create an environment variable "MODE" that can be use in other python scripts. I use it in the supervisor and the SAC controller for the training or the evaluation.
<br> <br>

## I revised "supervisor.py" to do a simpler world (less spawn attempts for the boxes) for the training part
<br> <br>

## I revised "sac_agent.py" & "replay_buffer.py" to store every variables as torch type, so that we don't have to convert each time the sac_agent want to use the replay buffer (only one conversion at the initialization of the replay buffer now).
<br> <br>

## I added the Main Loop (Train & Eval) in the "sac.py" controller. Things probably aren't perfect; for example I don't use "train_step" ("agent_sac.py methode) at every steps of the simulation (I saw that in several other examples) but I'm not sure the frequency that I need to put (every 10 steps, 5 steps, 3 steps). This is made to update the Models less frequently, so that we have less computation to do in a single simulation (episode) -> so we can do more episodes for the same training time, but the drawback is that learn less in a single episode. Maybe one other problem is the reward computation, I think that I need to reconsider the constant reward values for each field (maybe less time punition, better toward goal reward, ...), or maybe I can do a better computation reward considering other factors (more complex ones, as total distance travelled, no progression time, ...).
<br>