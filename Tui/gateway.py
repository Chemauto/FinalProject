from Planner.llm_core import chat, stream_chat

class LocalChatGateway:
    def send_chat(self, messages):
        return chat(messages)

    def stream_chat(self, messages):
        return stream_chat(messages)
#本地网关，先直接调用llm_core
