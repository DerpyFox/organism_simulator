# organism_simulator
Обучение агентов с физическим телом и зрением на Unity ML-Agents

# Порядок установки 

1. Установить ML-agents по инструкции, указанной в официальном репозитории https://github.com/Unity-Technologies/ml-agents/blob/develop/docs/Installation.md
2. После установки по инструкции, запустить в Anaconda powershell виртуальное окружение с папки проекта 

conda activate "название окружения"

3. Запустить Unity Hub, открыть проект Project (версия 6000.0.43f1)
4. В Unity Editor найти папку CrawlerVisual6Legs, открыть нужную сцену (CrawlerVisual6LegsWalker/CrawlerVisual6LegsSearcher)
5. Для запуска моделей на Inference нажать кнопку PLAY в сцене (проверить, что Behavior type стоит Inference only и назначена модель)
6. Для запуска обучения Walker, в Anaconda powershell выполнить команду

mlagents-learn "путь до файла конфигурации"/CrawlerVisual6LegsWalker.yaml --run-id="имя"

7. Для запуска обучения Searcher, назначить в Prefabs для CrawlerVisual6LegsSearcher в clild объекте CrawlerVisual6LegsWalker модель и выбрать Behavior Type - Inference Only. 
После этого в Anaconda powershell:

mlagents-learn "путь до файла конфигурации"/CrawlerVisual6LegsSearcher.yaml --run-id="имя"