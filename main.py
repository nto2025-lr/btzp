from flask import Flask, jsonify, send_from_directory, render_template, url_for
import logging
import subprocess
import cv2
import numpy as np

app = Flask(__name__, static_folder='static')

try:
    open('static/image.png')
except FileNotFoundError:
    cv2.imwrite('static/image.png', np.ones((800, 800, 3), dtype=np.uint8) * 255)
    
# Отключение спама в консоль
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
# Словарь с состоянием кнопок, по названию понятно вроде
button_state = {
    "start": True,
    "stop": False,
    "kill": True,
    "home": False,
    "land": False
}
script = None
# HTML
@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

# Фронтенд
@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory('static', filename)

# Получения состояния кнопок
@app.route('/api/state', methods=['GET'])
def get_state():
    return jsonify(button_state)

@app.route('/foto/<string:filename>')
def show_image(filename):
    print(filename)
    return f'''<img src="{url_for('static', filename=f"{filename}.png")}"
           alt="здесь">'''

# Получение текста из otchet.txt
@app.route('/api/buildings_text', methods=['GET'])
def get_buildings_text():
    try:
        with open('static/otchet.txt', 'r') as file:
            buildings_text = file.read()
        return jsonify({"text": buildings_text})
    except FileNotFoundError:
        return jsonify()

# Получение текста из info.txt
@app.route('/api/info_text', methods=['GET'])
def get_info_text():
    try:
        with open('static/info.txt', 'r') as file:
            info_text = file.read()
        return jsonify({"text": info_text})
    except FileNotFoundError:
        return jsonify()


# Обработка кнопки "START"
@app.route('/api/start', methods=['POST'])
def handle_start():
    global script
    if button_state["start"]:
        button_state["start"] = False
        button_state["stop"] = True
        button_state["land"] = True
        button_state["home"] = True
        script = subprocess.Popen(['python3', 'ros_script.py'])
    return jsonify(button_state)

# Обработка кнопки "LAND"
@app.route('/api/land', methods=['POST'])
def handle_land():
    global script
    if button_state["land"]:
        button_state["land"] = False
        button_state["stop"] = True
        button_state["kill"] = True
        button_state["home"] = True
        if script is not None:
            script.kill()
            script = None
        subprocess.Popen(['python3', 'ros_script.py', '--land'])
    return jsonify(button_state)

# Обработка кнопки "KILL SWITCH"
@app.route('/api/kill', methods=['POST'])
def handle_kill():
    global script
    if script is not None:
        script.kill()
        script = None
    subprocess.Popen(['python3', 'ros_script.py', '--kill'])
    button_state["stop"] = False
    button_state["start"] = False
    button_state["home"] = False
    button_state["land"] = False
    return jsonify(button_state)

# Обработка кнопки "STOP"
@app.route('/api/stop', methods=['POST'])
def handle_stop():
    global script
    subprocess.Popen(['python3', 'ros_script.py', '--stop'])
    button_state["stop"] = False
    button_state["start"] = True
    button_state["home"] = True
    button_state["land"] = True
    return jsonify(button_state)

# Обработка кнопки "HOME"
@app.route('/api/home', methods=['POST'])
def handle_home():
    global script
    subprocess.Popen(['python3', 'ros_script.py', '--home'])
    button_state["stop"] = True
    button_state["start"] = True
    button_state["home"] = False
    button_state["land"] = True
    return jsonify(button_state)

if __name__ == "__main__":
    # Запуск сервера через localhost на порте 8888
    print("Сервер запущен на http://192.168.50.120:8888/")
    app.run(host='192.168.50.120', port=8888, debug=False)

