
theta1Range = range(-90, 90, 10)
theta2Range = range(-90, 90, 10)

for theta1 in theta1Range:
	for theta2 in theta2Range:
		theta3 = -90 + (-theta1 + 180 - theta2)
		print(str(theta1) + "\t" + str(theta2) + "\t" + str(theta3))



theta1 = -32
theta2 = 66
theta3 = -90 + (-theta1 + 180 - theta2)
print(str(theta1) + "\t" + str(theta2) + "\t" + str(theta3))
		