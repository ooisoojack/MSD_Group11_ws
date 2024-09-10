import os
import time
import requests

BOT_TOKEN = "7299492934:AAGoFfGdU0dZTac3eVQP5byGTK6_98EAhsQ"

url = f"https://api.telegram.org/bot{BOT_TOKEN}/getUpdates"

chat_id = "-1002259258276"

msg = "fk utar"
url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage?chat_id={chat_id}&text={msg}"


while(True):
    r = requests.get(url)
    print(r.json())
# print(requests.get(url).json())

# def main():
#     while(1):
#         chat_id = bot.getUpdates()[-1].message.chat_id
#         bot.sendMessage(chat_id=chat_id, text="Lmao")
#         time.sleep(5)

# if __name__ == "__main__":
#     main()