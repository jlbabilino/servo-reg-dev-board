import asyncio

from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout


async def print_garbage():
    x = 0
    while True:
        print(f"Hello {x}")
        await asyncio.sleep(1)
        x += 1

async def accept_stuff():
    session = PromptSession()

    while True:
        user_input = await session.prompt_async("Enter command > ")
        print("Echo: ", user_input)


async def main():
    with patch_stdout():
        await asyncio.gather(print_garbage(), accept_stuff())



if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Ctrl+C Detected. Shutting down...")