class BinarySingal:
    def __init__(
        self, signal_fct, state_list: list = [False, True], init_state=None
    ) -> None:
        self.last_signal = signal_fct()
        self.current_signal = None
        self.signal_fct = signal_fct
        self.index = 0 if init_state is None else state_list.index(init_state)
        self.state_list = state_list

    def update(self) -> None:
        self.current_signal = self.signal_fct()
        if self.current_signal and not self.last_signal:
            self.index += 1
            self.index %= len(self.state_list)
        self.last_signal = self.current_signal

    def get_state(self):
        return self.state_list[self.index]

    def get_current_signal(self):
        return self.current_signal