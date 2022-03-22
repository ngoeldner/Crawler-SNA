# Camera Calibration

Nesse diretório está o código utilizado para calibrar uma câmera.

A pasta [**`logitech_camera`**](./logitech_camera) contém imagens de exemplo que que são utilizadas para fazer a calibração.

Para obter os parâmetros intrínsecos de uma câmera (realizar a calibração), basta executar o arquivo [**`calibration.py`**](./calibration.py).
Para calibrar uma câmera qualquer, deve-se bater fotos do [**`CameraCalibrationGrid.pdf`**](./CameraCalibrationGrid.pdf) impresso; 
armazenar as fotos em uma pasta e mudar o caminho para as novas imagens no código (atualmente o caminho está para as imagens de exemplo).

O arquivo `CameraCalibrationGrid.docs` é onde o criado foi criado. O tamanho de cada célula pode ser alterado, se isso for desejado.

Já o arquivo `CameraCalibrationGrid.pdf` é o grid pronto para ser impresso em folha A4.

Os arquivos gerados `dist.npy` e `mtx.npy` apresentam, respectivamente, os parâmetros de distorção e os parâmetros intrínsecos da câmera. 
