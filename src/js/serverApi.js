import { API_URL } from "./constants.js";

export const getBacklinksByContentId = async (contentId) => {
  const personData = await fetch(`${API_URL}/backlinks/byContentId/${contentId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}
export const getForwardlinksByBacklinkId = async (backlinkId) => {
  const personData = await fetch(`${API_URL}/forwardlinks/byBacklinkId/${backlinkId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}
export const getNodeByContentId = async (ContentId) => {
  const personData = await fetch(`${API_URL}/nodeByContentId/${ContentId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}
export const getNodeByBacklinkId = async (backlinkId) => {
  const personData = await fetch(`${API_URL}/nodeByBacklinkId/${backlinkId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}
export const searchWiki = async (inputVal) => {
  const personData = await fetch(`${API_URL}/search/${inputVal}`);
  const jsonResult = await personData.json();
  console.log(jsonResult);
  return jsonResult
}
export const getPersonImage = async (contentId) => {
  const personData = await fetch(`${API_URL}/imageByContentId/${contentId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  if(rowCount === 1){
    const {image_name, image_subdir} = rows[0]
    const imgResponse = await fetch(`${API_URL}/${image_subdir}/${image_name}`)
    const blobResult = await imgResponse.blob();
    return blobResult;
  }
  return null;
}