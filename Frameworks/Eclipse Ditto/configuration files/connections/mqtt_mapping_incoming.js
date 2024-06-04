function mapToDittoProtocolMsg(headers, textPayload, bytePayload, contentType) {
    const payload = String.fromCharCode.apply(null, new Uint8Array(bytePayload));
    const topic  = headers["mqtt.topic"].split('/')
    const value = { 
        value: { 
            properties: { 
                value: payload
            } 
        } 
    };    
    return Ditto.buildDittoProtocolMsg(
        topic[1]+'.'+topic[0], // namespace 
        topic[2], 
        'things', 
        'twin',
        'commands', 
        'modify',
        '/features', 
        headers, 
        value
    );
}