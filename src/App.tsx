import React from 'react';
import styled from 'styled-components';
// import {ForceGraph2D} from 'react-force-graph';
import Graph2D from './Graph2D'
import {getBacklinksByContentId} from './js/serverApi.js';
import {mkNetworkData} from './js/dataHandlers.js';
import './App.css'

const Container = styled.div``
const RefreshButton = styled.button`
  position:  absolute;
  top: 0;
  right: 0;
  z-index: 10;  
`

const COLORS = {
  person: 'yellow',
  other: 'grey'
}
const contentId = '가수_한국_C_004301_유재석';
const initialNode = {
  id:  contentId,
  text: '유재석',
  color: COLORS.person,
  value: 100
}

function App() {
  const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[initialNode], links:[]});
  // get initial network data
  React.useEffect( () => {
    getBacklinksByContentId(contentId)
    .then((result: unknown) => {
      console.log(result)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = mkNetworkData(result, contentId, lastNetworkData);
        return newNetworkData;
      })
    })
  }, [])
  const handleNodeClick = React.useCallback(async (node)  => {
    const {id, isPerson} = node;
    if(!isPerson){
      return false
    }
    console.log(id, isPerson)
    const rows = await getBacklinksByContentId(id)
    const newNetworkData = mkNetworkData(rows, node.id, lastNetworkData);
    setLastNetworkData(newNetworkData)
  }, [lastNetworkData]);

  return (
    <Container>
      <Graph2D
        graphData={lastNetworkData}
        handleNodeClick={handleNodeClick}
      ></Graph2D>
    </Container>
  )
}

export default App
