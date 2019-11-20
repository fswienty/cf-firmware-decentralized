
scfs = {
    'radio://0/80/2M/E7E7E7E7E4': 'four',
    'radio://0/80/2M/E7E7E7E7E9': 'nine'
}




print(scfs)

print("######################")
inp = input()
inp = inp.split()
print("######################")
for uri in scfs.keys():
    if uri[-1] is inp[0]:
        print(scfs[uri])

