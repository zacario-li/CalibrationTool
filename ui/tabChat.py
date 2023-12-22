import wx
import wx.html2
import markdown
import os
import threading
import json
import cv2
from multiprocessing import Pool 
import numpy as np
from loguru import logger
from langchain.chat_models.tongyi import ChatTongyi
from langchain.schema import HumanMessage
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import StructuredOutputParser
from getpass import getpass 

DASHSCOPE_API_KEY = "sk-cedc6aac7e9948b5a335185ef042eb44"

# chatLLM = ChatTongyi(dashscope_api_key=DASHSCOPE_API_KEY, streaming=True)
# res = chatLLM.stream([HumanMessage(content="总结下今天的天气")])
# for r in res:
#     print("chat resp:",r)

class TabChat():
    def __init__(self, parent, tab):
        self.tab = tab
        self.init_chat()
        self.init_ui()        
        self.update_display()  # 初始化聊天显示

    def init_ui(self):
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        # 使用 WebView 而不是 TextCtrl
        self.web_view = wx.html2.WebView.New(self.tab)
        self.input_text_ctrl = wx.TextCtrl(self.tab, style=wx.TE_PROCESS_ENTER)
        self.send_button = wx.Button(self.tab, label="Send")

        self.sizer.Add(self.web_view, 5, wx.EXPAND | wx.ALL, 5)
        self.sizer.Add(self.input_text_ctrl, 0, wx.EXPAND | wx.ALL, 5)
        self.sizer.Add(self.send_button, 0, wx.EXPAND | wx.ALL, 5)

        self.tab.SetSizer(self.sizer)

        self.tab.Bind(wx.EVT_BUTTON, self.on_send_button, self.send_button)
        self.tab.Bind(wx.EVT_TEXT_ENTER, self.on_send_button, self.input_text_ctrl)

        # 定义CSS样式
        self.css_styles = """
        <style>
            .chat-container {
                padding: 0 10px;
            }
            .user-message {
                text-align: right;
                max-width: 80%;
                margin-left: 20%;
                word-wrap: break-word;
            }
            .bot-message {
                text-align: left;
                max-width: 80%;
                margin-right: 20%;
                word-wrap: break-word;
            }
        </style>
        """

        self.chat_content = "<html><head {self.css_styles}></head><body>"  # 存储聊天内容的HTML

    def init_chat(self):
        self.chatLLM = ChatTongyi(dashscope_api_key=DASHSCOPE_API_KEY, streaming=True)

    def on_send_button(self, event):
        user_input = self.input_text_ctrl.GetValue().strip()
        if user_input:
            self.input_text_ctrl.Clear()
            self.append_to_chat(f"You: {user_input}\n", is_markdown=False)
            thread = threading.Thread(
                target=self._run_chatbot_stream, args=(user_input,)
            )
            thread.start()
    
    def _run_chatbot_stream(self, user_input):
        response = self.get_chatbot_response(user_input)
        wx.CallAfter(self.append_to_chat, response, True, False)

    def on_clear_button(self, event):
        pass

    def append_to_chat(self, message, is_markdown=False, is_user=True):
        css_class = "user-message" if is_user else "bot-message"
        # 将Markdown转换为HTML
        message = markdown.markdown(message)
        self.chat_content += f'<div class="{css_class}">{message}</div>'
        self.update_display()

    def update_display(self):
        # 更新 WebView 中的内容
        self.web_view.SetPage(self.chat_content+"</body></html>", "")

    def get_chatbot_response(self, input_text):
        # 调用 ChatTongyi 进行聊天
        response = self.chatLLM.stream([HumanMessage(content=input_text)])
        return "[Assistant]:\n"+' '.join([r.content for r in response])