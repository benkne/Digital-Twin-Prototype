function mapFromDittoProtocolMsg(namespace,name,group,channel,criterion,action,path,dittoHeaders,value,status,extra) {
  let headers = dittoHeaders;
  let textPayload = value.toString();
  let bytePayload = null;
  let contentType = 'application/vnd.eclipse.ditto+json';
  return Ditto.buildExternalMsg(
    headers, // The external headers Object containing header values
    textPayload, // The external mapped String
    bytePayload, // The external mapped byte[]
    contentType // The returned Content-Type
  );
}