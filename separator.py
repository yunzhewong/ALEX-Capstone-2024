import csv


def writeCVP(file, data, index):
    time = data[0]
    c = data[index * 3 + 1]
    v = data[index * 3 + 2]
    p = data[index * 3 + 3]

    file.write(f"{time}, {c}, {v}, {p}\n")


with open("correctintegral.csv", mode="r") as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)  # Read the first line as header (if present)

    abductor = open("abductor.csv", mode="w")
    extensor = open("extensor.csv", mode="w")
    knee = open("knee.csv", mode="w")

    # Iterate through the rows
    for data in csv_reader:
        time = data[0]

        writeCVP(abductor, data, 1)
        writeCVP(extensor, data, 3)
        writeCVP(knee, data, 5)

    abductor.close()
    extensor.close()
    knee.close()
