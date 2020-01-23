#include <nimbus_ros_driver/websocket.h>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

namespace nimbus{
    WebSocketClient::WebSocketClient(const std::string addr, bool continuousTrig, 
                    double port, double jsonPort,
                    int reconnectIntents, double imgBufSize, ros::NodeHandle& nh) : _nh(nh) 
    {
        this->_address = addr;
        this->_streamPort = port;
        this->_jsonPort = jsonPort;

        this->_reconnectIntents = reconnectIntents;

        this->_imgBufSize = imgBufSize;
        this->_listenStarted = false;
        this->_listenEnded = false;
        this->_connected = false;
        this->_disconnectMe = false;

        double c_ = 299792458;
        double fmod = 11.78e6;
        this->_UR = c_/(2 * fmod);

        // ..
        this->spread = this->getSpreadFactorXYZ<float_t>();
        std::vector<int16_t> unitX = this->getUnitVectorX<std::vector<int16_t>>();
        std::vector<int16_t> unitY = this->getUnitVectorY<std::vector<int16_t>>();
        std::vector<int16_t> unitZ = this->getUnitVectorZ<std::vector<int16_t>>();
        
        // ToDo: Remove the tidious for loop!!
        for(size_t i = 0; i < 286*352; i++) this->_uX[i] = unitX[i] / spread;
        for(size_t i = 0; i < 286*352; i++) this->_uY[i] = unitY[i] / spread;
        for(size_t i = 0; i < 286*352; i++) this->_uZ[i] = unitZ[i] / spread;
        this->_imageQueue.push("");
        this->connect();  
    }

    WebSocketClient::~WebSocketClient()
    {
        this->endpoint.close(web_id, web_close, "close");
        this->_listenThread.join();
    }

    std::vector<std::vector<float>> WebSocketClient::getImage(){
        //ToDo poll queue to get the data
        std::vector<std::vector<float> > myVec;
        while((this->_connected == false) || (_queueFront == "")){
            if(!this->_imageQueue.empty()) 
                this->_queueFront = this->_imageQueue.front();  // Wait for to estblish the connection
        }
        if((this->_connected != false) && (_queueFront != ""))
        {
            this->_queueFront = this->_imageQueue.front();
            ImageDecoded imgDecoded = this->create(_queueFront);
            int imgType = imgDecoded.header[HeaderImgType];
            if (this->imagType != 0)
            {
                imgType = 70;
                // ToDo apmlitude
                std::vector<float> radialO;
                for(size_t i =0; i < 286*352; i++) radialO.push_back((float)this->radials[i]/65535*this->_UR);
                for(int i = 0; i < 286*352; i++)
                {
                    int tempC = this->confidence[i];
                    if(tempC == 1) radialO[i] = 0;
                }
                //ToDo implement invalid as nan with conf 
                if(!(imgType & NimbusImageX))
                {
                    std::vector<float> tempX;
                    for(size_t i = 0; i < 286*352; i++)
                        tempX.push_back(radialO[i] * this->_uX[i]);
                    myVec.push_back(tempX);
                }
                if(!(imgType & NimbusImageY))
                {
                    std::vector<float> tempY;
                    for(size_t i = 0; i < 286*352; i++)
                        tempY.push_back(radialO[i] * this->_uY[i]);
                    myVec.push_back(tempY);
                }
                if(!(imgType & NimbusImageZ))
                {   
                    std::vector<float> tempZ;
                    for(size_t i = 0; i < 286*352; i++)
                        tempZ.push_back(radialO[i] * this->_uZ[i]);
                    myVec.push_back(tempZ);
                }
                //delete[] radial;
            }
        }
        return myVec;
    }

    void WebSocketClient::listenerThread()
    {
        while(!this->_imageQueue.empty())
            this->_imageQueue.pop();
        int intent = 0;
        while (intent < this->_reconnectIntents && _nh.ok())
        {
            std::string status = metadata.get()->get_status();
            if (status == "Open")
            {
                if(this->_imageQueue.size() <= this->_imgBufSize){
                    this->_connected = true;
                    this->_imageQueue.push(metadata.get()->_queue);
                }else{
                    while(!(this->_imageQueue.size() <= this->_imgBufSize))
                        this->_imageQueue.pop();
                    this->_imageQueue.push(metadata.get()->_queue);
                    this->_connected = true;
                }
            }else if(status == "Connecting")
            {
                this->_connected = false;
                std::cout << "Connecting to the Web Socket Please wait:...!" << std::endl;
            }else if(status == "Failed"){
                this->_connected = false;
                std::cout << "Failed to connect .. Trying to reconnect again" << std::endl;
                intent +=1;
                //ToDo without exiting this thread this->connect();
            }
            
        }
    }

    void WebSocketClient::connect()
    {
        _streamURL = "ws://" + _address + ":" + std::to_string(_streamPort) + "/stream";
        std::cout << "Web socket stream address: " << _streamURL.c_str() << std::endl;
        this->web_id = endpoint.connect(_streamURL.c_str());
        metadata = endpoint.get_metadata(this->web_id);
        if (metadata) {
            std::cout << metadata.get() << std::endl;
        } else {
            std::cout << "> Unknown connection id " << web_id << std::endl;
        }

        try
        {
            this->_listenThread = std::thread(&WebSocketClient::listenerThread,  this);
        }
        catch(const std::runtime_error &e)
        {
            std::cout << e.what() << std::endl;
        }  

    }

    template<typename D>
    D WebSocketClient::getUnitVectorX(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 4, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            int16_t* my_vec_x = (int16_t*)decode.c_str();
            std::vector<int16_t> temp;
            for(int i = 0; i < 286*352; i++)
                temp.push_back(my_vec_x[i]);
            resultValue =  temp;
        }
        return (resultValue);
    }
    template<typename D>
    D WebSocketClient::getUnitVectorY(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 5, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            int16_t* my_vec_y = (int16_t*)decode.c_str();
            std::vector<int16_t> temp;
            for(int i = 0; i < 286*352; i++)
                temp.push_back(my_vec_y[i]);
            resultValue =  temp;
        }
        return (resultValue);
    }
    template<typename D>
    D WebSocketClient::getUnitVectorZ(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 6, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            int16_t* my_vec_z = (int16_t*)decode.c_str();
            std::vector<int16_t> temp;
            for(int i = 0; i < 286*352; i++)
                temp.push_back(my_vec_z[i]);
            resultValue =  temp;
        }
        return (resultValue);
    }

    template<typename D>
    D WebSocketClient::getSpreadFactorXYZ()
    {
        D returnDouble;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 7, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0"){
            std::string data(result_r["result"].asString());
            double dResult = std::stod(data);
            returnDouble = dResult;
        }
        return (returnDouble);
    }

    size_t WebSocketClient::WriteCallback(const char* in, std::size_t size, std::size_t num, std::string* out)
    {
        const std::size_t totalBytes(size * num);
        out->append(in, totalBytes);
        return totalBytes;
    }

    template<typename T, typename D>
    T WebSocketClient::_getJsonParameter(D data)
    {
        CURL *curl = curl_easy_init();
        struct curl_slist *headers = NULL;
        CURLcode res;
        std::string readBuffer;
        _jsonURL = "http://" + _address + ":" + std::to_string(_jsonPort) + "/jsonrpc";
        std::cout << "JSON RPC stream address: " << _jsonURL.c_str() << std::endl;
        unsigned char * url = (unsigned char *)_jsonURL.c_str();       //ToDo Change this to string
        long httpCode(0);
        std::unique_ptr<std::string> httpData(new std::string());
        const std::string result_string;
        T resultJson;

        if(curl)
        {
            /** ToDo:
             * @brief Use the json paeser construct that payload and use this as string!!
             */
            const char *payload = data; 
            headers = curl_slist_append(headers, "content-type: application/json;");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_URL, url);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) strlen(payload));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &this->WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());

            res = curl_easy_perform(curl);
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
            if(res != CURLE_OK){
                fprintf(stderr, "curl_easy_persform() failed: %s\n", curl_easy_strerror(res));
            }
            curl_easy_cleanup(curl);
        }

        if(httpCode == 200)
        {
            Json::Value jsonData;
            Json::Reader jsonReader;
            if(jsonReader.parse(*httpData.get(), jsonData))
            {
                jsonReader.parse(jsonData["result"].toStyledString(), jsonData);
                const std::string dataString(jsonData["result"].asString());
                resultJson = jsonData;
            }else
            {
                std::cout << "Could not parse HTTP data as JSON" << std::endl;
                std::cout << "HTTP data was:\n" << *httpData.get() << std::endl;
                return "error";
            }
            
        }else
        {
            std::cout << "Couldn't GET from " << url << " - exiting" << std::endl;
            return "error";
        }
        curl_global_cleanup();
        return (resultJson);
    }

  

    
    float * WebSocketClient::unpack(std::string buf){
        float * b = (float_t *) buf.c_str();
        return (b);
    }

    /** Interpret the input from Web socket*/
    
    ImageDecoded WebSocketClient::create(std::string buffer)
    {
        ImageDecoded imgDecoded;
        std::string temp = std::string(buffer.begin(), buffer.begin() + 8);
        float * Size = this->unpack(temp);
        int headerSize = (int)Size[1];
        temp = std::string(buffer.begin(), buffer.begin() + headerSize);
        float * headers = (float *)temp.c_str();
        int imgType = (int)headers[HeaderImgType];
        int width = (int)headers[HeaderROIWidth];
        int height = (int)headers[HeaderROIHeight];
        int numSeq = (int)headers[HeaderNumSequences];
        this->imagType =imagType;
        if(imgType == NimbusImageRaw)
        {
            /** @todo: 
             * 1. decode the hole buffer into uint16_t array
             * 2. arr = arr.reshape((numSeqs, height, width))
            */
        }else{
            int imgSize = height * width * 2;
            int confSize = height * width * 1;
            int amplStart = headerSize;
            int amplStop = amplStart + imgSize;
            if (imgType & NimbusImageAmpl)
            {
                temp = std::string(buffer.begin()+amplStart, buffer.begin()+amplStop);
                uint16_t * amplt = (uint16_t *)temp.c_str();                             //Reshape the array with Width and Height
                int count = 0;
                for(int i = 0; i< height; i++)                  //ROW
                {
                    std::vector<float> vecTemp;
                    for(int j = 0; j < width; j++)              //COLUMN
                    {
                        vecTemp.push_back((float)amplt[count]);
                        count ++;
                    }
                    this->amplitude.push_back(vecTemp);
                }
            }
            else{} // ToDo 
            int radialStart = amplStop;
            int radialStop = radialStart + imgSize;
            if(imgType & NimbusImageDist)
            {
                temp = std::string(buffer.begin()+radialStart, buffer.begin()+radialStop);
                uint16_t * radials = (uint16_t *)temp.c_str();                           //Reshape the array with Width and Height
                imgDecoded.radial = radials;
                while(!this->radials.empty()) this->radials.pop_back();
                for(int i = 0; i < width * height; i++) this->radials.push_back(radials[i]);
                
            }
            else{} //ToDo
            int confStart = radialStop;
            int confStop = confStart + confSize;
            if(imgType & NimbusImageConf)
            {
                temp = std::string(buffer.begin()+confStart, buffer.begin()+confStop);
                uint8_t * conf = (uint8_t *)temp.c_str();                                //Reshape the array with Width and Height
                while(!this->confidence.empty()) this->confidence.pop_back();
                for(int i = 0; i<width * height; i++)
                {
                    //std::cout <<"Confidence at: " << i << " is: " << (int)conf[i] << std::endl;
                    this->confidence.push_back((int)conf[i]);
                }
                    
            }
            else{} //ToDo
            int xStart = confStop;
            int xStop = xStart + imgSize;
            if(imgType & NimbusImageX){
                temp = std::string(buffer.begin()+xStart, buffer.begin()+xStop);
                int16_t * xD = (int16_t *)temp.c_str();                                       //Reshape the array with Width and Height
            }
            else {}//ToDo
            int yStart = confStop;
            int yStop = yStart + imgSize;
            if(imgType & NimbusImageY){
                temp = std::string(buffer.begin()+yStart, buffer.begin()+yStop);
                int16_t * yD = (int16_t *)temp.c_str();                                       //Reshape the array with Width and Height
            }
            else{} //ToDo
            int zStart = confStop;
            int zStop = zStart + imgSize;
            if(imgType & NimbusImageZ){
                temp = std::string(buffer.begin()+zStart, buffer.begin()+zStop);
                int16_t * zD = (int16_t *)temp.c_str();                                       //Reshape the array with Width and Height
            }
            else{} //ToDo
        }
        imgDecoded.header = headers;
        return imgDecoded;
    }

}
