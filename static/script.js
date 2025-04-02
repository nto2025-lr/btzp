document.addEventListener('DOMContentLoaded', () => {
    const startButton = document.getElementById('start');
    const stopButton = document.getElementById('stop');
    const killButton = document.getElementById('kill');
    const landButton = document.getElementById('land');
    const homeButton = document.getElementById('home');
    const imageElement = document.querySelector('.image-container img');
    const buildingsTextElement = document.getElementById('buildings-text');
    const infoTextElement = document.getElementById('info-text');

    const baseUrl = '/api';

    const updateButtons = (response) => {
        startButton.disabled = !response.start;
        stopButton.disabled = !response.stop;
        landButton.disabled = !response.land;
        homeButton.disabled = !response.home;
        killButton.textContent = `KILL SWITCH`;
    };

    const sendRequest = async (button) => {
        try {
            const response = await fetch(`${baseUrl}/${button}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                }
            });
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            updateButtons(data);
        } catch (error) {
            console.error('Error:', error);
        }
    };

    startButton.addEventListener('click', () => {
        if (!startButton.disabled) sendRequest('start');
    });

    stopButton.addEventListener('click', () => {
        if (!stopButton.disabled) sendRequest('stop');
    });

    landButton.addEventListener('click', () => {
        if (!landButton.disabled) sendRequest('land');
    });

    homeButton.addEventListener('click', () => {
        if (!homeButton.disabled) sendRequest('home');
    });

    killButton.addEventListener('click', () => sendRequest('kill'));

    // Получаение начального состояния 
    fetch(`${baseUrl}/state`)
        .then(response => response.json())
        .then(updateButtons)
        .catch(error => console.error('Error:', error));

    // Функция для обновления изображения
    const updateImage = () => {
        const timestamp = new Date().getTime();
        imageElement.src = `image.png?t=${timestamp}`;
    };

    //  Обновление изображения каждые 0.2 сек
    setInterval(updateImage, 200);

    // Функция для бновления текста
    const updateVariableText = async () => {
        try {
            const response = await fetch(`${baseUrl}/buildings_text`, {
                method: 'GET',
                headers: {
                    'Content-Type': 'application/json'
                }
            });
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            buildingsTextElement.innerHTML = data.text.replace(/\n/g, '<br>');
        } catch (error) {
            console.error('Error:', error);
        }
    };

    // Обновление текста каждые 0.2 сек
    setInterval(updateVariableText, 200);

    // Функция для бновления текста
    const updateVariableTextInfo = async () => {
        try {
            const response = await fetch(`${baseUrl}/info_text`, {
                method: 'GET',
                headers: {
                    'Content-Type': 'application/json'
                }
            });
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            infoTextElement.innerHTML = data.text.replace(/\n/g, '<br>');
        } catch (error) {
            console.error('Error:', error);
        }
    };

    // Обновление текста каждые 0.2 сек
    setInterval(updateVariableTextInfo, 200);
});
