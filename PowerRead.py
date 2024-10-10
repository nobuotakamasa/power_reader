import subprocess

#command = ["sudo","./tegrastats", "--interval", "1000" "--verbose"]
command = ["sudo","tegrastats", "--interval", "1000" "--verbose"]
process = subprocess.Popen(command, stdout = subprocess.PIPE, stderr=subprocess.PIPE)
while True:
	output = process.stdout.readline()
	output = output.strip()
	#output = str(output).split(' ')
	print(output)

