from unittest.mock import call

from commands.command import Command
from handlers.handler import Handler

ONCE_HANDLED_DATA = 'ONCE_HANDLED_DATA'
HANDLED_DATA = None


class StubHandler(Handler):
    def __init__(self, on_handle, on_unregister):
        self.on_handle = on_handle
        self.on_unregister = on_unregister

    def handle(self, handled_data):
        self.on_handle(handled_data)

        return ONCE_HANDLED_DATA

    def unregister(self):
        self.on_unregister()


def test_when_executing_then_handle(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')
    command = Command([StubHandler(on_handle_stub, on_unregister_stub)])

    command.execute(HANDLED_DATA)

    on_handle_stub.assert_called_once_with(None)
    on_unregister_stub.assert_called_once()


def test_given_multiple_handlers_when_executing_then_handle(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')
    command = Command([
        StubHandler(on_handle_stub, on_unregister_stub),
        StubHandler(on_handle_stub, on_unregister_stub)
    ])

    command.execute(HANDLED_DATA)

    on_handle_stub.assert_has_calls([call(None), call(ONCE_HANDLED_DATA)])
    assert on_unregister_stub.call_count == 2


def test_given_next_command_when_executing_then_pass_handled_data(mocker):
    on_handle_stub = mocker.stub(name='on_handle_stub')
    on_unregister_stub = mocker.stub(name='on_unregister_stub')

    commands = [Command([StubHandler(on_handle_stub, on_unregister_stub)]), Command([StubHandler(on_handle_stub, on_unregister_stub)])]

    handled_data = HANDLED_DATA
    for command in commands:
        handled_data = command.execute(handled_data)

    on_handle_stub.assert_has_calls([call(None), call(ONCE_HANDLED_DATA)])
    assert on_unregister_stub.call_count == 2
