import React from 'react';
import styled from 'styled-components';
import {getBacklinksByContentId} from '../js/serverApi.js';
import {mkNetworkData} from '../js/dataHandlers.js';

const RowContainer = styled.div`
  padding: 3px;
  background-color: black;
  margin-bottom: 3px;
`
const Node = styled.div`
  display: flex;
  font-size: 12px;
  font-weight: 100;
  color: yellow;
`
const Action = styled.div`
  margin-left: 3px;
`
const Title = styled(Action)`
  /* color: ${(props) => props.action === 'del' && 'lightgrey'}; */
  cursor: pointer;
  width: 100%;
  white-space: nowrap;
  text-overflow: ellipsis;
  overflow: hidden;
  &:hover {
    color: white;
  };
`
const DelButton = styled(Action)`
  color: red;
  margin-left: auto;
  font-weight: 200;
  cursor: pointer;
  &:hover {
    color: white;
  };
`
const TimeStamp = styled(Action)`
  font-size: 11px;
  color: #d73232;
  text-align: right;
  cursor: pointer;
`

function NodeBacklink(props) {
  const {
    node, 
    removeNode, 
    expandNode,
    setActiveExpandedNodeId
  } = props;
  const [isShowAll, setIsShowAll] = React.useState(false);
  const onClickTitle = React.useCallback(async (event) => {
    expandNode(node)
  }, [node])
  const toggleShowsAllBacklinks = React.useCallback((event) => {
    const {id} = event.target;
    setIsShowAll( async (isShowAll) => {
      const includeOnlyContents = isShowAll;
      const rows = await getBacklinksByContentId(id)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = mkNetworkData(rows, id, lastNetworkData, includeOnlyContents);
        return newNetworkData
      })
    })
  }, [])
  return (
    <RowContainer>
      <Node key={node.id}>
        <Title
          id={node.id}
          onClick={onClickTitle}
        >
          {node.text}
        </Title>
        <DelButton id={node.id} onClick={removeNode}>
          [Del]
        </DelButton>
      </Node>
      <TimeStamp
        onClick={toggleShowsAllBacklinks}
        id={node.id}
      >
        backlinks [{node.backlinkCount}]
      </TimeStamp>
    </RowContainer>
  )
}

export default React.memo(NodeBacklink)