import pygame
import sys

print("Initializing pygame...")
pygame.init()

print("Creating display...")
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Test Window")

print("Filling screen...")
screen.fill((255, 0, 0))  # Red background
pygame.display.flip()

print("Window should be visible now!")
print("Running event loop...")

running = True
count = 0
while running and count < 300:  # Run for 5 seconds (300 frames at 60fps)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    pygame.display.update()
    pygame.time.Clock().tick(60)
    count += 1

print("Closing...")
pygame.quit()
print("Done!")
