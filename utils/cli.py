import threading

class CLI(threading.Thread):
    def __init__(self):
        super().__init__(name="Teleoperation CLI")
        self._func_map = {"Q": lambda: False}
        self._instruction_map = {"Q": "Exit"}

    def register_function(self, key: str, label: str, fn: callable):
        _k = key.upper()
        self._instruction_map[_k] = label
        self._func_map[_k] = fn

    def remove_function(self, key: str):
        _k = key.upper()
        if _k in self._func_map:
            del self._instruction_map[_k]
            del self._func_map[_k]

    def print_instructions(self):
        print()
        for key, text in self._instruction_map.items():
            print("({}) {}".format(key, text))

    def exec_cmd(self, cmd: str):
        if cmd not in self._func_map:
            print("Wrong Command!")
            return True

        func = self._func_map[cmd]
        cli_continue = func()

        if cli_continue is None:
            cli_continue = True
        return cli_continue

    def get_input(self) -> bool:
        # self.print_instructions()
        cmd = str(input("Enter Command... ")).upper()
        return self.exec_cmd(cmd)

    def run(self):
        while self.get_input():
            continue
