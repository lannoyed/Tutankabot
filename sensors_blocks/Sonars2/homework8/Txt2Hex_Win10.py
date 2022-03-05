import sys, getopt, re

InputFileName = ""
OutputFileName = ""

def main(argv):
    global InputFileName
    global OutputFileName
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print("Txt2Hex.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("Txt2Hex.py -i <inputfile> -o <outputfile>")
            sys.exit()
        elif opt in ("-i", "--ifile"):
            InputFileName = arg
        elif opt in ("-o", "--ofile"):
            OutputFileName = arg


if __name__ == "__main__":
    main(sys.argv[1:])

    InputFile  = open(InputFileName, "r", encoding='utf-16-le')
    InputContent = InputFile.read()
    match_list = re.findall(r"(?<=^.{8}:\s).{8}",InputContent,re.MULTILINE)
    InputFile.close()
    
    OutputFile = open(OutputFileName, "w")
    OutputFile.writelines([match+"\n" for match in match_list])
    OutputFile.close()

