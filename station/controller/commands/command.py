class Command:
    def __init__(self, handlers, step=None):
        self.handlers = handlers
        self.step = step

    def execute(self, handled_data):
        if self.step is not None:
            handled_data["current_step_pub"].publish(self.step.value)

        for handler in self.handlers:

            handled_data = handler.handle(handled_data)

            handler.unregister()

        return handled_data
