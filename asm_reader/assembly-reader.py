# ==================================================================================================
# imports


# ==================================================================================================
# defines
DEBUG = False


# ==================================================================================================
# utility functions
def is_hex(string):
    try:
        int(string, 16)
        return True
    except ValueError:
        return False


# ==================================================================================================
# class definitions
class LineType:
    DISMISS = 0
    LABEL = 1
    INSTRUCTION = 2


class AssemblyReader:
    def __init__(self, file_path):
        self.file_path = file_path

        self.lines = self.read()
        self.parsed_data = self.parse()


    def read(self):
        with open(self.file_path, 'r') as f:
            return f.readlines()
    

    def parse(self):
        parsed_data = []
        for line in self.lines:
            line_data = line.strip().split()
            parsed_data.append(line_data)
        
        return parsed_data
    

    def determine_linetype(self, line_data):
        if len(line_data) == 2:
            return LineType.LABEL
        elif len(line_data) > 2 and is_hex(line_data[1]):
            return LineType.INSTRUCTION
        else:
            return LineType.DISMISS
    

    def get_instruction_total(self, vector_instructions_only = False):
        """Count the total number of instructions in the assembly file.
        """
        count = 0
        for line_data in self.parsed_data:
            if self.determine_linetype(line_data) == LineType.INSTRUCTION:
                if vector_instructions_only:
                    if line_data[2].startswith("v"):
                        count += 1
                else:
                    count += 1

        return count
    

    def get_instruction_list(self, vector_instructions_only = False):
        """List every unique instruction in the assembly file.
        """
        instructions = []
        for line_data in self.parsed_data:
            if self.determine_linetype(line_data) == LineType.INSTRUCTION:
                instructions.append(line_data[2])

        if vector_instructions_only:
            instructions = [instruction for instruction in instructions if instruction.startswith("v")]

        # alphabetize the instructions and remove duplicates
        instructions = sorted(instructions)

        return list(set(instructions))
    

    def get_instruction_count(self, vector_instructions_only = False):
        """Count the number of each unique instruction in the assembly file.
        """
        instructions = {}
        for line_data in self.parsed_data:
            if self.determine_linetype(line_data) == LineType.INSTRUCTION:
                if line_data[2] in instructions:
                    instructions[line_data[2]] += 1
                else:
                    instructions[line_data[2]] = 1
        
        if vector_instructions_only:
            instructions = {instruction: count for instruction, count in instructions.items() if instruction.startswith("v")}

        # alphabetize the instructions
        instructions = dict(sorted(instructions.items()))

        return instructions
    

    def print_instruction_stats(self, vector_instructions_only = False):
        """Print the statistics of the instructions in the assembly file.
        """
        filter_string = "Vector " if vector_instructions_only else ""
        
        # print the total number of instructions
        print(f"Total {filter_string}Instructions: {self.get_instruction_total(vector_instructions_only)}")

        # print the total number of unique instructions
        print(f"Total Unique {filter_string}Instructions: {len(self.get_instruction_list(vector_instructions_only))}")

        # print the number of each unique instruction
        print(f"{filter_string}Instruction Count:")
        for instruction, count in self.get_instruction_count(vector_instructions_only).items():
            print(f"  {instruction}: {count}")
        
        print()
    

    def get_subroutine_instructions(self, subroutine_name):
        """Get the instructions for a specific subroutine.
        """
        fetching = False
        subroutine_parsed_data = []
        for line_data in self.parsed_data:
            if self.determine_linetype(line_data) == LineType.LABEL:
                if line_data[1] == "<" + subroutine_name + ">:":
                    fetching = True
                elif fetching:
                    break
            if fetching and self.determine_linetype(line_data) == LineType.INSTRUCTION:
                subroutine_parsed_data.append(line_data)
        
        return subroutine_parsed_data

        
    
    def print_subroutine_instruction_stats(self, subroutine_name, vector_instructions_only = False):
        """Print the statistics of the instructions in a specific subroutine.
        """
        print(f"Subroutine '{subroutine_name}' Instruction Stats:")
        subroutine_parsed_data = self.get_subroutine_instructions(subroutine_name)

        temp_parsed_data = self.parsed_data

        self.parsed_data = subroutine_parsed_data

        self.print_instruction_stats(vector_instructions_only)

        self.parsed_data = temp_parsed_data

    
        




def main():
    reader = AssemblyReader("llama2_asm.txt")
    
    reader.print_instruction_stats(vector_instructions_only = True)

    reader.print_subroutine_instruction_stats("rmsnorm", vector_instructions_only = True)
    reader.print_subroutine_instruction_stats("softmax", vector_instructions_only = True)
    reader.print_subroutine_instruction_stats("matmul", vector_instructions_only = True)
    reader.print_subroutine_instruction_stats("forward", vector_instructions_only = True)


if __name__ == "__main__":
    main()