import numpy as np
import pygame
import random
import time
import math

pygame.init()

# Window size
window_width = 800
window_height = 600
window = pygame.display.set_mode((window_width, window_height))


pygame.display.set_caption("MRS Project - Blob Tag")
icon = pygame.image.load("icon.jpeg")
pygame.display.set_icon(icon)

p_image = pygame.image.load("pursuer.png")
e_image = pygame.image.load("escapee.png")

# Scal to fit the window size
pursuer_image = pygame.transform.scale(p_image, (50, 50))
escapee_image = pygame.transform.scale(e_image, (50, 50))


background_color = (255, 255, 255)
text_color = (0, 0, 0) 
font = pygame.font.SysFont("Arial", 32)


agents = []

class Agent:
    def __init__(self, x, y, role):
        self.x = x
        self.y = y
        self.role = role

        # HERE'S THE SPEED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if self.role == "pursuer":
            self.speed = 10 
        else:
            self.speed = 11 
        # SPEED IS IS PIXELS PER UPDATE

        self.direction = random.randint(0, 359)

    def update(self):
        dx = self.speed * math.cos(math.radians(self.direction))
        dy = self.speed * math.sin(math.radians(self.direction))

        # change current position
        self.x += dx
        self.y += dy

        # Checking if agent is out of bounds and bounc back if so
        if self.x < 0 or self.x > window_width - 50:
            self.x -= dx 
            self.direction = 180 - self.direction 

        if self.y < 0 or self.y > window_height - 50:
            self.y -= dy 
            self.direction = -self.direction 

    def draw(self):
        if self.role == "pursuer":
            window.blit(pursuer_image, (self.x, self.y)) 
        else:
            window.blit(escapee_image, (self.x, self.y)) 

    def distance(self, other):
        dx = self.x - other.x 
        dy = self.y - other.y 

        return math.sqrt(dx**2 + dy**2)

    def chase(self):
        if self.role == "pursuer":
            closest_escapee = None 
            min_distance = float("inf") # initialize minimum distance as infinity

            for agent in agents: 
                if agent.role == "escapee": 
                    distance = self.distance(agent) 

                    if distance < min_distance: # check if this is closest so far
                        closest_escapee = agent # updat the closest escapee
                        min_distance = distance 
            
            if closest_escapee: # check if there is any escapee left
                dx = closest_escapee.x - self.x 
                dy = closest_escapee.y - self.y 
                angle = math.degrees(math.atan2(dy, dx))
                self.direction = angle

    def escape(self):
        if self.role == "escapee":
            closest_pursuer = None
            min_distance = float("inf") 

            for agent in agents: 
                if agent.role == "pursuer": 
                    distance = self.distance(agent) # calculate the distance to the pursuer

                    if distance < min_distance: # check if this is closest pursuer till now
                        closest_pursuer = agent 
                        min_distance = distance 
            
            corner_region = 0
            
            if closest_pursuer: 
                dx = closest_pursuer.x - self.x 
                dy = closest_pursuer.y - self.y 

                # HEREEEEE'SSSS THE PRINBTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT !!!!!!!!!!!!!!!!!!!!!!
                # print(dx,dy)
                angle = math.degrees(math.atan2(dy, dx))

                xx = self.x 
                yy = self.y
                dist1 = math.sqrt((xx**2+yy**2))
                dist2 = math.sqrt((800-xx)**2+(yy)**2)
                dist3 = math.sqrt((xx)**2+(600-yy)**2)
                dist4 = math.sqrt((800-xx)**2+(600-yy)**2)
                self.direction = angle + 180 + np.random.uniform(25)               

                if dist1 <= 150:
                    corner_region = 1
                    del_ang = 270 - angle
                    if del_ang > 45:
                        self.direction = math.degrees(math.atan2((600-self.y),self.x)) - np.random.uniform(10)
                        # print("chiryo111111111111111111")
                    else:
                        self.direction = math.degrees(math.atan2(self.y,(800-self.x))) + np.random.uniform(25)
                        # print("chiryo222222222222222222222222222222222222")

                elif dist2 <= 150:
                    corner_region = 2

                    del_ang = angle - 270
                    if del_ang > 45:
                        self.direction = math.degrees(math.atan2((600-self.y),(800-self.x))) -100
                    else:
                        self.direction = math.degrees(math.atan2((self.y),(self.x))) +170


                elif dist3 <= 275:
                    corner_region = 3
                elif dist4 <= 275:
                    corner_region = 4
                else:
                    corner_region = 0

                
    def capture(self):
        if self.role == "pursuer":
            for agent in agents: 
                if agent.role == "escapee": 
                    distance = self.distance(agent) 
                    if distance < 50: 
                        agent.role = "pursuer" 
                        agent.speed = 10 
                        

for i in range(12):
    x = random.randint(0, window_width - 50) 
    y = random.randint(0, window_height - 50) 

    if i == 0: #FIRST AGENT PURSUER
        role = "pursuer"
    else: # rest escapees
        role = "escapee"

    agent = Agent(x, y, role)
    agents.append(agent) 
    
running = True


while running:
    window.fill(background_color)

    # Get all events from pygame
    events = pygame.event.get()

    for event in events: 
        if event.type == pygame.QUIT: 
            running = False 
    
    num_pursuers = 0 
    num_escapees = 0 

    for agent in agents: 
        agent.update() # updatES position
        agent.draw() 

        if agent.role == "pursuer": 
            num_pursuers += 1 # incrementing the number of pursuers by one
            agent.chase() 
        
        else:
            num_escapees += 1 # incrementing the number of escapees by one
            agent.escape() 
        
        agent.capture()
    # to print numnber of pursuers and escapees    
    # print(f"Number of pursuers: {num_pursuers}") 
    # print(f"Number of escapees: {num_escapees}") 

    for agent in agents: 
        if agent.role == "escapee":
            print(f"Escape velocity and direction of escapee at ({agent.x}, {agent.y}): {agent.speed} pixels per update and {agent.direction} degrees") 
            # printing escape velocity and direction
                
    pygame.display.update()
    
    time.sleep(0.09) 
    
    if num_escapees == 0: 
        running = False