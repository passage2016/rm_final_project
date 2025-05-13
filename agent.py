import os
import requests

from langchain_deepseek import ChatDeepSeek
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

os.environ["DEEPSEEK_API_KEY"] = "please input your api key"
os.environ["DEEPSEEK_BASE_URL"] = "https://api.deepseek.com/v1"

def ask_ai(prompt):
    url = "http://your_llm_host:11434/api/generate"
    model_name = "deepseek-r1:70b"
    data = {
        "model": model_name,
        "prompt": prompt,
        "stream": False
    }
    response = requests.post(url, json=data)
    return response.json()["response"]


def ask_deepseek(model_name, user_prompt):
    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", "你是一个AI助理，请帮助用户完成任务。"),
            ("user", "{input}")
        ]
    )
    deepseek_chat = ChatDeepSeek(model=model_name)
    chain = prompt | deepseek_chat | StrOutputParser()
    return chain.invoke(user_prompt)