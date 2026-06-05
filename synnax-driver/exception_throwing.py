import asyncio
import concurrent.futures
import traceback


async def worker_1():
    while True:
        print("worker 1")
        await asyncio.sleep(3)

async def worker_2():
    while True:
        print("worker 2")
        await asyncio.sleep(2)

async def main():
    print("Starting await")
    done, pending = await asyncio.wait([asyncio.create_task(worker_1()),
                                        asyncio.create_task(worker_2())],
                        return_when = concurrent.futures.FIRST_COMPLETED)
    print("Ending await")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt as e:
        traceback.print_exception(e)
