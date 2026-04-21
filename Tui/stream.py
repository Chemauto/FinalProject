class StreamItem:
    def __init__(self, item_type="assistant"):
        self.data = {"type": item_type, "content": "", "status": "streaming"}
    #创建流式消息

    def add(self, text):
        self.data["content"] += text
    #追加模型返回片段

    def done(self):
        self.data["status"] = "done"
    #标记流式输出完成
